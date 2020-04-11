//! DHCP Client + Data Server
#![allow(clippy::single_match)]
#![allow(clippy::option_map_unit_fn)]

use crate::ethernet::EthernetDMA;

use crate::http;
use crate::http::{HttpReceiveState, HttpServer};
use crate::omron_m2::ValveState;
use crate::sensors::SensorData;

use smoltcp::dhcp::Dhcpv4Client;
use smoltcp::iface::{
    EthernetInterface, EthernetInterfaceBuilder, Neighbor, NeighborCache,
    Route, Routes,
};
// use smoltcp::phy::wait as phy_wait;
use smoltcp::socket::{
    RawPacketMetadata, RawSocketBuffer, SocketHandle, SocketSet, SocketSetItem,
};
use smoltcp::socket::{TcpSocket, TcpSocketBuffer};
use smoltcp::time::{Duration, Instant};
use smoltcp::wire::{
    EthernetAddress, IpAddress, IpCidr, Ipv4Address, Ipv4Cidr,
};

// ---- Socket Storage -------------------------------

const DHCP_RX_BUFFER_SIZE: usize = 4096;
const DHCP_TX_BUFFER_SIZE: usize = 4096;
const TCP_RX_BUFFER_SIZE: usize = 8192;
const TCP_TX_BUFFER_SIZE: usize = 8192;

/// Buffer storage for a TCP Socket
struct TCPStorage {
    rx_storage: [u8; TCP_RX_BUFFER_SIZE],
    tx_storage: [u8; TCP_TX_BUFFER_SIZE],
}
impl TCPStorage {
    pub const fn new() -> Self {
        TCPStorage {
            rx_storage: [0; TCP_RX_BUFFER_SIZE],
            tx_storage: [0; TCP_TX_BUFFER_SIZE],
        }
    }
}
macro_rules! tcp_socket_from {
    ($store:ident, $name:ident) => {{
        let tcp_rx_buffer =
            TcpSocketBuffer::new(&mut $store.$name.rx_storage[..]);
        let tcp_tx_buffer =
            TcpSocketBuffer::new(&mut $store.$name.tx_storage[..]);
        TcpSocket::new(tcp_rx_buffer, tcp_tx_buffer)
    }};
}

// ---- Socket Pool Storage --------------------------

type TCPSocketPoolStorage = (TCPStorage, TCPStorage, TCPStorage);
macro_rules! new_tcp_socket_pool_storage {
    () => {
        (TCPStorage::new(), TCPStorage::new(), TCPStorage::new())
    };
}
/// A pool of TCP sockets
struct TCPSocketPool(pub [SocketHandle; 3]);

// Constructs a TCPSocketPool from element $name in store $store
macro_rules! tcp_socket_pool_into_from {
    ($sockets:ident, $store:ident, $name:ident, $($sockN:tt),+) => {{
        TCPSocketPool([
            $(
                $sockets.add({
                    let tcp_rx_buffer =
                        TcpSocketBuffer::new(
                            &mut $store.$name.$sockN.rx_storage[..]);
                    let tcp_tx_buffer =
                        TcpSocketBuffer::new(
                            &mut $store.$name.$sockN.tx_storage[..]);

                    TcpSocket::new(tcp_rx_buffer, tcp_tx_buffer)
                })
            ),+
        ])
    }};
    ($sockets:ident, $store:ident, $name:ident) => {
        tcp_socket_pool_into_from!($sockets, $store, $name, 0, 1, 2)
    }
}
// Applies a work command for each socket in a TCPSocketPool
macro_rules! tcp_socket_pool_work {
    ($self:ident, $work_command:ident, $pool:ident
     => $($sockN:tt),+) => {
        $(
            $self.$work_command($self.$pool.0[$sockN]);
        )+
    };
    ($self:ident, $work_command:ident, $pool:ident, $arg:ident
     => $($sockN:tt),+) => {
        $(
            $self.$work_command($self.$pool.0[$sockN], $arg);
        )+
    };
    ($self:ident, $work_command:ident, $pool:ident $(, $arg:ident)*) => {
        tcp_socket_pool_work!($self, $work_command, $pool
                              $(, $arg)* => 0, 1, 2)
    }
}

// ---- Server Storage -------------------------------

/// Stack allocated storage for dataserver
pub struct DataServerStorage<'a, 'b> {
    ip_addrs: [IpCidr; 1],
    socket_set_entries: [Option<SocketSetItem<'a, 'b>>; 8],
}

impl<'a, 'b> DataServerStorage<'a, 'b> {
    pub fn new() -> Self {
        DataServerStorage {
            ip_addrs: [IpCidr::new(Ipv4Address::UNSPECIFIED.into(), 0)],
            socket_set_entries: Default::default(),
        }
    }
}

/// Statically allocated storage for the dataserver
pub struct DataServerStorageStatic {
    neighbor_cache_storage: [Option<(IpAddress, Neighbor)>; 8],
    routes_storage: [Option<(IpCidr, Route)>; 1],

    dhcp_rx_meta_storage: [RawPacketMetadata; 1],
    dhcp_tx_meta_storage: [RawPacketMetadata; 1],
    dhcp_rx_storage: [u8; DHCP_RX_BUFFER_SIZE],
    dhcp_tx_storage: [u8; DHCP_TX_BUFFER_SIZE],

    // Ports
    command: TCPSocketPoolStorage,
    streaming: TCPSocketPoolStorage,
}
impl DataServerStorageStatic {
    pub const fn new() -> Self {
        DataServerStorageStatic {
            neighbor_cache_storage: [None; 8],
            routes_storage: [None; 1],

            dhcp_rx_meta_storage: [RawPacketMetadata::EMPTY; 1],
            dhcp_tx_meta_storage: [RawPacketMetadata::EMPTY; 1],
            dhcp_rx_storage: [0; DHCP_RX_BUFFER_SIZE],
            dhcp_tx_storage: [0; DHCP_TX_BUFFER_SIZE],

            // Ports
            command: new_tcp_socket_pool_storage!(),
            streaming: new_tcp_socket_pool_storage!(),
        }
    }
}

pub struct DataServer<'a: 'b, 'b, VT, MT>
where
    VT: Fn(ValveState),
    MT: Fn(u16),
{
    iface: EthernetInterface<'a, 'a, 'a, EthernetDMA<'a>>,
    sockets: SocketSet<'a, 'a, 'b>,
    dhcp: Dhcpv4Client,
    http: HttpServer<VT, MT>,
    ip_cidr: Ipv4Cidr,
    router: Ipv4Address,
    command_tcp: TCPSocketPool,
    streaming_tcp: TCPSocketPool,
}

impl<'a, 'b, VT, MT> DataServer<'a, 'b, VT, MT>
where
    VT: Fn(ValveState),
    MT: Fn(u16),
{
    pub fn new(
        store: &'static mut DataServerStorageStatic,
        dstore: &'a mut DataServerStorage<'a, 'b>,
        ethdev: EthernetDMA<'a>,
        ethernet_addr: EthernetAddress,
        http: HttpServer<VT, MT>,
        now: i64,
    ) -> Self {
        // Static storage may be in a RAM not initialised by the
        // runtime.
        store.neighbor_cache_storage = [None; 8];
        store.routes_storage = [None; 1];

        // Neighbor cache
        let neighbor_cache =
            NeighborCache::new(&mut store.neighbor_cache_storage[..]);
        // Routing table
        let routes = Routes::new(&mut store.routes_storage[..]);

        // Interface
        let iface = EthernetInterfaceBuilder::new(ethdev)
            .ethernet_addr(ethernet_addr)
            .neighbor_cache(neighbor_cache)
            .ip_addrs(&mut dstore.ip_addrs[..])
            .routes(routes)
            .finalize();

        // Socket set
        let mut sockets = SocketSet::new(&mut dstore.socket_set_entries[..]);

        // DHCP
        let dhcp_rx_buffer = RawSocketBuffer::new(
            &mut store.dhcp_rx_meta_storage[..],
            &mut store.dhcp_rx_storage[..],
        );
        let dhcp_tx_buffer = RawSocketBuffer::new(
            &mut store.dhcp_tx_meta_storage[..],
            &mut store.dhcp_tx_storage[..],
        );
        let dhcp = Dhcpv4Client::new(
            &mut sockets, // Adds RawSocket to socket set
            dhcp_rx_buffer,
            dhcp_tx_buffer,
            Instant::from_millis(now),
        );

        // TCP
        let command_tcp = tcp_socket_pool_into_from!(sockets, store, command);

        let streaming_tcp =
            tcp_socket_pool_into_from!(sockets, store, streaming);

        DataServer {
            iface,
            sockets,
            dhcp,
            http,
            ip_cidr: Ipv4Cidr::new(Ipv4Address::UNSPECIFIED, 0),
            router: Ipv4Address::UNSPECIFIED,
            command_tcp,
            streaming_tcp,
        }
    }

    fn work_command_server(&mut self, sock: SocketHandle) {
        let http = &self.http;
        let mut socket = self.sockets.get::<TcpSocket>(sock);
        if !socket.is_open() {
            socket.listen(80).unwrap();
        }

        // Storage for outgoing headers
        let mut headers = http::HttpResponseHeaders::new();

        // Process incoming
        if socket.can_recv() {
            let builder = http::HttpResponseBuilder::new(&mut headers);

            // Note that we don't consume partial HTTP packets
            let response = socket
                .recv(|buffer| http.server_recv(buffer, builder))
                .unwrap();

            // Send response
            match (socket.can_send(), &response) {
                (true, HttpReceiveState::Response(r)) => {
                    for slice in &r.response {
                        socket.send_slice(slice).unwrap();
                    }
                }
                _ => {}
            }

            // Update server state
            self.http.server_update(response);
        }

        // If remote closed
        if socket.may_send() && !socket.may_recv() {
            info!("tcp:80 close");
            socket.close();
        }
    }
    fn work_streaming_server(
        &mut self,
        sock: SocketHandle,
        data: Option<(usize, &[u8])>,
    ) {
        let mut socket = self.sockets.get::<TcpSocket>(sock);
        if !socket.is_open() {
            socket.listen(8080).unwrap();
        }

        // If socket is open, send data
        match (socket.can_send(), data) {
            (true, Some((len, buffer))) => {
                socket.send_slice(&buffer[..len]).unwrap();
            }
            _ => {}
        }

        // If remote closed
        if socket.may_send() && !socket.may_recv() {
            info!("tcp:8080 close");
            socket.close();
        }
    }

    /// Work on our sockets
    pub fn work(&mut self, now: i64, data: Option<SensorData>) -> Duration {
        let timestamp = Instant::from_millis(now);

        self.iface
            .poll(&mut self.sockets, timestamp)
            .map(|_| ())
            .unwrap_or_else(|e| info!("Poll: {:?}", e));

        // DHCP
        let config = self
            .dhcp
            .poll(&mut self.iface, &mut self.sockets, timestamp)
            .unwrap_or_else(|e| {
                info!("DHCP: {:?}", e);
                None
            });

        config.map(|config| {
            info!("DHCP config: {:?}", config);
            if let Some(cidr) = config.address {
                if cidr != self.ip_cidr {
                    self.iface.update_ip_addrs(|addrs| {
                        addrs.iter_mut().next().map(|addr| {
                            *addr = IpCidr::Ipv4(cidr);
                        });
                    });
                    self.ip_cidr = cidr;
                    info!("Assigned a new IPv4 address: {}", cidr);
                }
            }

            config.router.map(|router| {
                self.router = router;
                self.iface
                    .routes_mut()
                    .add_default_ipv4_route(router)
                    .unwrap()
            });
            self.iface.routes_mut().update(|routes_map| {
                routes_map
                    .get(&IpCidr::new(Ipv4Address::UNSPECIFIED.into(), 0))
                    .map(|default_route| {
                        info!("Default gateway: {}", default_route.via_router);
                    });
            });

            if config.dns_servers.iter().any(|s| s.is_some()) {
                info!("DNS servers:");
                for dns_server in config.dns_servers.iter().filter_map(|s| *s) {
                    info!("- {}", dns_server);
                }
            }
        });

        // Serialise outgoing data
        let mut buffer = [0u8; 256];
        let data_ser = match data {
            Some(data) => match data.serialise_to_buffer(&mut buffer) {
                Ok(len) => Some((len, &buffer[..])),
                Err(e) => {
                    warn!("Failed to serialise data : {}", e);
                    None
                }
            },
            _ => None,
        };

        // Ports
        tcp_socket_pool_work!(self, work_command_server, command_tcp);
        tcp_socket_pool_work!(
            self,
            work_streaming_server,
            streaming_tcp,
            data_ser
        );

        // Timeout is duration to instant when we next need to egress
        let mut timeout = self.dhcp.next_poll(timestamp);
        // A socket may have a shorter duration to next poll
        if let Some(sockets_timeout) =
            self.iface.poll_delay(&self.sockets, timestamp)
        {
            timeout = sockets_timeout;
        }

        timeout
    }

    /// Call to signal an interface state change
    pub fn interface_updown(&mut self, now: i64) {
        let timestamp = Instant::from_millis(now);

        // Reset DHCP
        self.dhcp.reset(timestamp);

        // Configure a 0.0.0.0 address on the interface
        self.iface.update_ip_addrs(|addrs| {
            addrs.iter_mut().next().map(|addr| {
                *addr = IpCidr::new(Ipv4Address::UNSPECIFIED.into(), 0);
            });
        });

        // Reset IP addresses
        self.ip_cidr = Ipv4Cidr::new(Ipv4Address::UNSPECIFIED, 0);
        self.router = Ipv4Address::UNSPECIFIED;

        // Abort any open TCP sockets
        for sock in self.command_tcp.0.iter() {
            self.sockets.get::<TcpSocket>(*sock).abort();
        }
    }

    /// Return the device's current IP v4 Address
    pub fn ipv4_address(&self) -> Ipv4Cidr {
        self.ip_cidr
    }

    /// Return the device's current gateway/router
    pub fn router(&self) -> Ipv4Address {
        self.router
    }
}
