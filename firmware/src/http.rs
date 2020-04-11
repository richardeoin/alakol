//! HTTP Server
use core::fmt::Write;
use core::str;

use alloc::{string::String, vec::Vec};

use crate::omron_m2::ValveState;

/// HTTP Constants
static HTTP_OK: &[u8] = b"HTTP/1.0 200 OK\r\n";
static INDEX_PAGE: &[u8] = include_bytes!("index.html");

static HTTP_SEE_OTHER: &[u8] = b"HTTP/1.0 303 See other\r\n";

static HTTP_BAD_REQUEST: &[u8] = b"HTTP/1.0 400 Bad request\r\n";
static BAD_REQUEST_PAGE: &[u8] = include_bytes!("400.html");

static HTTP_NOT_FOUND: &[u8] = b"HTTP/1.0 404 File not found\r\n";
static NOT_FOUND_PAGE: &[u8] = include_bytes!("404.html");

/// HTTP server
pub struct HttpServer<ValveT, MotorT>
where
    ValveT: Fn(ValveState),
    MotorT: Fn(u16),
{
    valve_fn: ValveT,
    motor_fn: MotorT,
    partial_counter: usize,
}
/// Response builder
pub struct HttpResponseBuilder<'buf, 'header: 'buf> {
    response: Vec<&'buf [u8]>,
    headers: &'header mut [String; 4],
}

/// A response from HTTP server
pub struct HttpResponse<'buf> {
    pub response: Vec<&'buf [u8]>,
}
impl<'b> HttpResponse<'b> {
    /// Creates a new response and clears it
    fn new_clear(mut response: Vec<&'b [u8]>) -> Self {
        response.clear();
        Self { response }
    }
    /// Push buffer onto response
    fn push(&mut self, buf: &'b [u8]) {
        self.response.push(buf)
    }
}

/// Headers for response
pub struct HttpResponseHeaders {
    headers: [String; 4],
}
impl HttpResponseHeaders {
    pub fn new() -> Self {
        Self {
            headers: [
                String::new(),
                String::new(),
                String::new(),
                String::new(),
            ],
        }
    }
}

/// HTTP builder errors
enum HttpError {
    FormattingError,
    ResponseError,
    RequestError,
}
impl From<core::fmt::Error> for HttpError {
    fn from(_: core::fmt::Error) -> Self {
        Self::FormattingError
    }
}
impl From<&[u8]> for HttpError {
    fn from(_: &[u8]) -> Self {
        Self::ResponseError
    }
}

/// HTTP Receive status
pub enum HttpReceiveState<'b> {
    Response(HttpResponse<'b>),
    Partial,
    None,
}
impl<'b> From<Result<HttpResponse<'b>, HttpError>> for HttpReceiveState<'b> {
    fn from(
        result: Result<HttpResponse<'b>, HttpError>,
    ) -> HttpReceiveState<'b> {
        use HttpReceiveState::*;
        match result {
            Ok(r) => Response(r),
            Err(_) => None,
        }
    }
}

macro_rules! match_kv_pair {
    ($body:ident, $name:ident => $key_str:expr, $self:ident, $func:ident) => {
        // Find this key in body
        match $body.find($key_str) {
            Some($name) => {
                let val = $body[$name..]
                    .split("&")
                    .nth(0)
                    .ok_or(HttpError::RequestError)?
                    .split("=")
                    .nth(1)
                    .ok_or(HttpError::RequestError)?;
                $self.$func(val)?;

                Some($name)
            }
            None => None,
        }
    };
}

/// Server
impl<VT, MT> HttpServer<VT, MT>
where
    VT: Fn(ValveState),
    MT: Fn(u16),
{
    pub fn new(valve: VT, motor: MT) -> Self {
        Self {
            valve_fn: valve,
            motor_fn: motor,
            partial_counter: 0,
        }
    }

    /// Update server state
    pub fn server_update<'b>(&mut self, state: HttpReceiveState<'b>) {
        use HttpReceiveState::*;

        // Increment consecutive partial counter
        match state {
            Partial => self.partial_counter += 1,
            _ => self.partial_counter = 0,
        };

        info!("partial count {}", self.partial_counter);
    }

    /// Return the body of a request
    fn get_request_body<'a>(
        &self,
        data: &'a [u8],
    ) -> Result<&'a str, HttpError> {
        let utf8 = unsafe { str::from_utf8_unchecked(data) };

        match (utf8.find("\r\n\r\n"), utf8.find("\n\n")) {
            (Some(crlf), _) => Ok(&utf8[crlf + 4..]),
            (_, Some(lf)) => Ok(&utf8[lf + 2..]),
            (_, _) => Err(HttpError::RequestError),
        }
    }

    /// Motor speed instruction
    fn motor_speed(&self, speed: &str) -> Result<(), HttpError> {
        info!("speed {}", speed);

        match (speed, speed.parse::<u16>()) {
            ("off", _) => {
                (self.motor_fn)(0);
                Ok(())
            }
            (_, Ok(speed)) if speed <= 100 => {
                (self.motor_fn)(speed);
                Ok(())
            }
            _ => Err(HttpError::RequestError),
        }
    }

    /// Valve state instruction
    fn valve_state(&self, valve: &str) -> Result<(), HttpError> {
        info!("valve {}", valve);

        match valve {
            "closed" => (self.valve_fn)(ValveState::Closed),
            "open" => (self.valve_fn)(ValveState::Open),
            _ => {}
        };

        Ok(())
    }

    /// Processes POST data
    fn process_post_data(
        &self,
        _path: &str,
        data: &[u8],
    ) -> Result<(), HttpError> {
        info!("process post data");
        let body = self.get_request_body(data)?;

        info!("body {}", body);

        let values = (
            match_kv_pair!(body, speed => "motor-speed=", self, motor_speed),
            match_kv_pair!(body, valve => "valve-state=", self, valve_state),
        );

        match values {
            // No values found =>
            (None, None) => Err(HttpError::RequestError),
            _ => Ok(()),
        }
    }

    /// Receive data from http socket
    ///
    /// Constructs response using headers allocated in the builder
    pub fn server_recv<'b, 'h>(
        &self,
        data: &mut [u8],
        builder: HttpResponseBuilder<'b, 'h>,
    ) -> (usize, HttpReceiveState<'b>) {
        if !data.is_empty() {
            debug!(
                "tcp recv data: {:?}",
                core::str::from_utf8(data).unwrap_or("(invalid utf8)")
            );

            // Parse HTTP
            let mut headers = [httparse::EMPTY_HEADER; 16];
            let mut req = httparse::Request::new(&mut headers);
            let result = req.parse(data).unwrap();

            if result.is_complete() {
                info!("{}", req.method.unwrap());

                // Complete
                let response = match (req.method, req.path) {
                    (Some("GET"), Some(path)) => {
                        builder.server_get(path).into() // GET
                    }
                    (Some("POST"), Some(path)) => {
                        builder.server_post(self, path, data).into() // POST
                    }
                    _ => HttpReceiveState::None,
                };

                (data.len(), response)
            } else if result.is_partial() && self.partial_counter < 5 {
                info!("partial");
                (0, HttpReceiveState::Partial) // Keep data
            } else {
                info!("invalid");
                // Invalid
                (data.len(), HttpReceiveState::None) // Drop data
            }
        } else {
            (0, HttpReceiveState::None)
        }
    }
}

/// A response from this server
impl<'b, 'h> HttpResponseBuilder<'b, 'h> {
    pub fn new(headers: &'h mut HttpResponseHeaders) -> Self {
        // We now own a reference to header storage that lives longer
        // than our buffer storage
        Self {
            response: Vec::new(),
            headers: &mut headers.headers,
        }
    }

    /// Called on HTTP POST
    ///
    /// Builds response using storage in supplied headers
    fn server_post<VT, MT>(
        self,
        server: &HttpServer<VT, MT>,
        path: &str,
        data: &[u8],
    ) -> Result<HttpResponse<'b>, HttpError>
    where
        VT: Fn(ValveState),
        MT: Fn(u16),
    {
        let mut resp = HttpResponse::new_clear(self.response);

        let (code, http) = match path {
            "/command" | "/command.html" => {
                match server.process_post_data(path, data) {
                    Ok(_) => (303, HTTP_SEE_OTHER), // See other 303
                    Err(_) => (400, HTTP_BAD_REQUEST), // Bad request 400
                }
            }
            _ => (404, HTTP_NOT_FOUND),
        };
        let body = match code {
            400 => Some(BAD_REQUEST_PAGE),
            404 => Some(NOT_FOUND_PAGE),
            _ => None,
        };

        // Response code
        resp.push(http);

        // Server
        self.headers[0].write_fmt(format_args!(
            "Server: smoltcp/alakol-{}-{}\r\n",
            super::build_info::PKG_VERSION,
            super::build_info::GIT_VERSION.unwrap(),
        ))?;

        // Content Length
        let len = match body {
            Some(body) => body.len(),
            None => 0,
        };
        self.headers[1]
            .write_fmt(format_args!("Content-Length: {}\r\n", len))?;

        if code == 303 {
            self.headers[2].write_fmt(format_args!("Location: /\r\n"))?;
        }

        // Headers
        resp.push(self.headers[0].as_bytes());
        resp.push(self.headers[1].as_bytes());
        if code == 303 {
            resp.push(self.headers[2].as_bytes());
        }

        // Body
        resp.push(b"\r\n");
        if let Some(body) = body {
            resp.push(body);
        }

        Ok(resp)
    }

    /// Called on HTTP GET
    ///
    /// Builds response using storage in supplied headers
    fn server_get(self, path: &str) -> Result<HttpResponse<'b>, HttpError> {
        let mut resp = HttpResponse::new_clear(self.response);

        let (http, body) = match path {
            "/" | "/index.html" => (HTTP_OK, INDEX_PAGE), // root
            _ => (HTTP_NOT_FOUND, NOT_FOUND_PAGE),        // 404
        };

        // Response code
        resp.push(http);

        // Server
        self.headers[0].write_fmt(format_args!(
            "Server: smoltcp/alakol-{}-{}\r\n",
            super::build_info::PKG_VERSION,
            super::build_info::GIT_VERSION.unwrap(),
        ))?;

        // Content Length
        self.headers[1]
            .write_fmt(format_args!("Content-Length: {}\r\n", body.len()))?;

        // Headers
        resp.push(self.headers[0].as_bytes());
        resp.push(self.headers[1].as_bytes());

        // Body
        resp.push(b"\r\n");
        resp.push(body);

        Ok(resp)
    }
}
