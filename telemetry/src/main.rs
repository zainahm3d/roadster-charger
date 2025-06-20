use chrono::{DateTime, Utc};
use influxdb::{Client, InfluxDbWriteable, WriteQuery};
use std::{io::BufRead, time::Duration};

#[derive(Clone, Debug, InfluxDbWriteable)]
pub struct State {
    time: DateTime<Utc>,

    tick: u32,
    mode: String,
    tick_disabled: u32,

    board_temp_c: i16,

    target_ma: u32,
    duty: u16,

    input_mv: u32,
    input_ma: u32,

    output_mv: u32,
    output_ma: u32,

    pub pdo_mv: u32,
    pub pdo_ma: u32,
}

#[tokio::main]
async fn main() {
    let token = std::env::var("INFLUX_TOKEN").unwrap();
    let ip = std::env::var("INFLUX_IP").unwrap();
    let client = Client::new(ip, "roadster").with_token(token);

    let port_name = find_port().expect("ESP32 not found");
    println!("ESP32 @ {}", port_name);

    // Baudrate doesn't matter because this port is CDC
    let port = serialport::new(port_name, 1_000_000)
        .timeout(Duration::from_millis(1000))
        .open()
        .expect("Failed to open port");

    let mut reader = std::io::BufReader::new(port);

    let mut frames: Vec<WriteQuery> = std::vec![];
    loop {
        let mut line = String::new();
        reader.read_line(&mut line).unwrap();

        // do a very illegal thing- use a debug printed struct as a data source
        if line.contains("&s = State {") {
            let mut buf: Vec<u8> = std::vec![];
            reader.read_until(b'}', &mut buf).unwrap();
            let message = String::from_utf8(buf).unwrap();
            let message = message.strip_suffix('}').unwrap();

            let frame = parse_status_message(&message);
            dbg!(&frame);
            frames.push(frame.into_query("roadster"));
        } else {
            print!("{}", line.trim()); // pass through normal print statements
        }

        if frames.len() >= 20 {
            if client.query(&frames).await.is_err() {
                if frames.len() >= 1000 {
                    frames.clear();
                    println!("Too many failed write attempts");
                    println!("\x07"); // terminal bell
                    std::process::exit(1);
                } else {
                    println!("Write failed, continuing...");
                }
            } else {
                frames.clear();
            }
        }
    }
}

fn find_port() -> Option<String> {
    let ports = serialport::available_ports().expect("No ports found");
    for p in ports {
        if let serialport::SerialPortType::UsbPort(usb_port) = p.port_type {
            if usb_port
                .manufacturer
                .is_some_and(|m| m.eq_ignore_ascii_case("espressif"))
            {
                return Some(p.port_name);
            }
        }
    }
    None
}

// This parser is temporary- future work includes sending the bare struct from the charger
// instead of a stringified form like this. Readable info is good for debugging right now.
fn parse_status_message(message: &str) -> State {
    let mut status = State {
        time: chrono::Utc::now(),
        tick: 0,
        mode: "".to_string(),
        tick_disabled: 0,
        board_temp_c: 0,
        target_ma: 0,
        duty: 0,
        input_mv: 0,
        input_ma: 0,
        output_mv: 0,
        output_ma: 0,
        pdo_mv: 0,
        pdo_ma: 0,
    };

    for line in message.split(",\n") {
        if line.contains(":") {
            let line = line.trim();

            let pair: Vec<&str> = line.split(':').collect();
            assert!(pair.len() == 2);

            let key = pair[0].trim().to_lowercase();
            let value = pair[1].trim().to_lowercase();

            match key.as_str() {
                "tick" => status.tick = value.parse().unwrap(),
                "mode" => status.mode = value.parse().unwrap(),
                "tick_disabled" => status.tick_disabled = value.parse().unwrap(),
                "board_temp_c" => status.board_temp_c = value.parse().unwrap(),
                "target_ma" => status.target_ma = value.parse().unwrap(),
                "duty" => status.duty = value.parse().unwrap(),
                "input_mv" => status.input_mv = value.parse().unwrap(),
                "input_ma" => status.input_ma = value.parse().unwrap(),
                "output_mv" => status.output_mv = value.parse().unwrap(),
                "output_ma" => status.output_ma = value.parse().unwrap(),
                "pdo_mv" => status.pdo_mv = value.parse().unwrap(),
                "pdo_ma" => status.pdo_ma = value.parse().unwrap(),

                _ => {
                    panic!("Unknown key in status message")
                }
            }
        }
    }
    status
}
