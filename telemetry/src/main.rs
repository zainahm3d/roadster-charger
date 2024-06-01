use chrono::{DateTime, Utc};
use influxdb::{Client, InfluxDbWriteable, WriteQuery};
use std::{io::BufRead, time::Duration};

#[derive(Clone, Debug, InfluxDbWriteable)]
struct ChargeController {
    time: DateTime<Utc>,

    system_state: String,
    charger_mode: String,

    board_temp_c: i16,

    tick: u32,

    target_ma: u32,
    target_mv: u32,

    p_gain: i32,
    p_error: i32,
    p_term: i32,

    input_mv: u32,
    input_ma: u32,

    output_mv: u32,
    output_ma: u32,
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
        if line.starts_with("ChargeController {") {
            let frame = parse_status_line(&line);
            dbg!(&frame);
            frames.push(frame.into_query("roadster"));
        } else {
            print!("{}", line); // pass through normal print statements
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

// This parser is temporary- future works includes sending the bare struct from the charger
// instead of a stringified form like this.
fn parse_status_line(line: &str) -> ChargeController {
    let mut status = ChargeController {
        time: chrono::Utc::now(),

        system_state: "".to_string(),
        charger_mode: "".to_string(),

        board_temp_c: 0,

        tick: 0,

        target_ma: 0,
        target_mv: 0,

        p_gain: 0,
        p_error: 0,
        p_term: 0,

        input_mv: 0,
        input_ma: 0,
        output_mv: 0,
        output_ma: 0,
    };

    let mut line = line.strip_prefix("ChargeController {").unwrap().to_string();
    line = line.strip_suffix("}\n").unwrap().to_string();

    for item in line.split(',') {
        let pair: Vec<&str> = item.split(':').collect();
        assert!(pair.len() == 2);

        let key = pair[0].trim().to_lowercase();
        let value = pair[1].trim().to_lowercase();

        match key.as_str() {
            "system_state" => status.system_state = value.parse().unwrap(),
            "charger_mode" => status.charger_mode = value.parse().unwrap(),
            "board_temp_c" => status.board_temp_c = value.parse().unwrap(),
            "tick" => status.tick = value.parse().unwrap(),
            "target_ma" => status.target_ma = value.parse().unwrap(),
            "target_mv" => status.target_mv = value.parse().unwrap(),
            "p_gain" => status.p_gain = value.parse().unwrap(),
            "p_error" => status.p_error = value.parse().unwrap(),
            "p_term" => status.p_term = value.parse().unwrap(),
            "input_mv" => status.input_mv = value.parse().unwrap(),
            "input_ma" => status.input_ma = value.parse().unwrap(),
            "output_mv" => status.output_mv = value.parse().unwrap(),
            "output_ma" => status.output_ma = value.parse().unwrap(),
            _ => {
                panic!("Unknown key in status message")
            }
        }
    }
    status
}
