from time import perf_counter_ns, time
import csv
import pathlib
import queue
import threading
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from tkinter.scrolledtext import ScrolledText

import serial
from serial.tools import list_ports
import pyvisa
import nidaqmx
import nidaqmx.constants


SAMPLE_INTERVAL_S = 1.0
SPIN_MARGIN_S = 0.02 #busy-wait this long before each deadline; must exceed OS sleep granularity (~15.6ms on Windows)
TORR_TO_PA = 133.3223684211
DEFAULT_DIR = "./data_output"

COMMAND_PRESSURE = "#01RD\r" #01 (ID) ReaD \r (carraige return)
COMMAND_RFPOWER = "Q\r"
COMMAND_SETTOSERIAL = "***\r" #Set Seren R301 to serial mode for reading forward and reflected power
COMMAND_STOP = "S\r"
COMMAND_GO = "G\r" #run the R301's programmed routine

PRESSURE_PORT = 'COM2'
RF_PORT = 'COM4'
CAMERA_PORT = None #Arduino camera trigger (camera_trigger.ino); None = auto-detect, or set e.g. 'COM5'
CAMERA_BAUD = 115200
CAMERA_READY_TIMEOUT_S = 3.0 #the Arduino resets when the port opens; bootloader + setup() takes ~2 s
COMMAND_CAMERA_TRIGGER = "T\r" #fire the camera trigger (only fires once until re-armed)
COMMAND_CAMERA_RESET = "R\r" #re-arm the camera trigger

#(csv column, label, unit, display format) - order is the CSV column order
PARAMETERS = [
    ("time_s", "Time", "s", "{:.1f}"),
    ("rough_pressure_pa", "Rough pressure", "Pa", "{:.4g}"),
    ("fine_pressure_pa", "Fine pressure", "Pa", "{:.4g}"),
    ("voltage_v", "Voltage", "V", "{:.6g}"),
    ("forward_power_w", "Forward power", "W", "{}"),
    ("reflected_power_w", "Reflected power", "W", "{}"),
]
CSV_HEADER = [key for key, _, _, _ in PARAMETERS]


def now_s():
    return perf_counter_ns() / 1e9


def sleep_until(target_s, stop_event):
    #sleep() can overshoot by a full OS timer tick, so sleep until just before the target, then spin to hit it exactly
    #stop_event.wait() is used instead of sleep() so a stop request wakes the loop immediately
    remaining = target_s - now_s()
    if remaining > SPIN_MARGIN_S and stop_event.wait(remaining - SPIN_MARGIN_S):
        return
    while now_s() < target_s and not stop_event.is_set():
        pass


def default_file_name():
    return f"voltage_pressure_run_{int(time())}"


def find_camera_port():
    #first port that looks like an Arduino, skipping the ports used by the other instruments
    for p in list_ports.comports():
        if p.device in (PRESSURE_PORT, RF_PORT):
            continue
        desc = f"{p.device} {p.description} {p.manufacturer or ''}".lower()
        if any(k in desc for k in ("usbmodem", "usbserial", "arduino", "ch340", "wch")):
            return p.device
    return None


def quietly(fn):
    #run a cleanup step, ignoring failures so the remaining steps still run
    try:
        fn()
    except Exception:
        pass


class Instruments:
    def __init__(self):
        self.task = None
        self.ser_pressure = None
        self.ser_rfpower = None
        self.rmpyvisa = None
        self.voltage_meter = None
        self.ser_camera = None

    def open(self, log):
        #open nidaqmx analog channels
        self.task = nidaqmx.task.Task()
        self.task.ai_channels.add_ai_voltage_chan("Dev1/ai0", terminal_config=nidaqmx.constants.TerminalConfiguration.RSE)

        #Open serial to rough pressure sensor and Seren R301 RF generator
        self.ser_pressure = serial.Serial(PRESSURE_PORT, baudrate=1200, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=0.15)
        self.ser_rfpower = serial.Serial(RF_PORT, baudrate=19200, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=0.15)
        self.ser_rfpower.write(COMMAND_SETTOSERIAL.encode('utf-8'))

        #Open pyvisa to Agilent 34401A voltmeter
        self.rmpyvisa = pyvisa.ResourceManager()
        self.voltage_meter = self.rmpyvisa.open_resource('USB0::0x0957::0x1A07::MY53201512::INSTR')

        self.open_camera(log)

    def open_camera(self, log):
        #the camera trigger is optional: if it can't be opened, data collection still runs without it
        port = CAMERA_PORT or find_camera_port()
        if port is None:
            log("WARNING: Camera trigger not found, recording without camera sync. Set CAMERA_PORT if it is connected.")
            return
        try:
            self.ser_camera = serial.Serial(port, CAMERA_BAUD, timeout=0.1)
        except serial.SerialException as e:
            log(f"WARNING: Could not open camera trigger on {port}, recording without camera sync. Code: {e}")
            return
        deadline = now_s() + CAMERA_READY_TIMEOUT_S
        while now_s() < deadline:
            if self.ser_camera.readline().strip() == b"READY":
                break
        else:
            log(f"WARNING: No READY from camera trigger on {port}; continuing anyway.")
        self.ser_camera.reset_input_buffer()
        log(f"Camera trigger connected on {port}")

    def trigger_camera(self, log):
        #returns True if the trigger command was sent; the reply is checked later by camera_reply() so the first sample isn't delayed
        if self.ser_camera is None:
            return False
        try:
            self.ser_camera.reset_input_buffer()
            self.ser_camera.write(COMMAND_CAMERA_TRIGGER.encode('utf-8'))
            return True
        except serial.SerialException as e:
            log(f"WARNING: Camera trigger failed. Code: {e}")
            return False

    def start_rf_routine(self, log):
        #returns True if the go command was sent
        try:
            self.ser_rfpower.write(COMMAND_GO.encode('utf-8'))
            return True
        except serial.SerialException as e:
            log(f"WARNING: Could not send go to RF power supply. Code: {e}")
            return False

    def stop_rf_routine(self, log):
        try:
            self.ser_rfpower.write(COMMAND_STOP.encode('utf-8'))
            log("RF routine stopped")
        except serial.SerialException as e:
            log(f"WARNING: Could not send stop to RF power supply. Code: {e}")

    def camera_reply(self):
        try:
            return self.ser_camera.readline().decode('utf-8', errors='replace').strip()
        except serial.SerialException as e:
            return f"error: {e}"

    def reset_camera(self, log):
        if self.ser_camera is None:
            return
        try:
            self.ser_camera.write(COMMAND_CAMERA_RESET.encode('utf-8'))
            self.ser_camera.flush() #make sure R is sent before the port is closed
            log("Camera trigger reset")
        except serial.SerialException as e:
            log(f"WARNING: Camera trigger reset failed. Code: {e}")

    def close(self):
        #only close what was actually opened, so a failed open() still cleans up
        if self.task is not None:
            quietly(self.task.close)
        if self.ser_rfpower is not None:
            quietly(lambda: self.ser_rfpower.write(COMMAND_STOP.encode('utf-8'))) #Stop the R301 from running
            quietly(self.ser_rfpower.close)
        if self.ser_pressure is not None:
            quietly(self.ser_pressure.close)
        if self.rmpyvisa is not None:
            quietly(self.rmpyvisa.close)
        if self.ser_camera is not None:
            quietly(self.ser_camera.close)

    def read(self, log):
        #DAQ and voltmeter failures raise and stop acquisition; pressure/RF failures are logged and recorded as "NA"
        pressure_voltage_direct = self.task.read()
        if isinstance(pressure_voltage_direct, (list, tuple)): #task.read() can return a list; take the single sample
            pressure_voltage_direct = pressure_voltage_direct[0]
        pressure_voltage_direct = float(pressure_voltage_direct)
        voltage = self.voltage_meter.query_ascii_values('MEAS:VOLT:DC?')[0]

        try:
            self.ser_pressure.write(COMMAND_PRESSURE.encode('utf-8'))
            raw_response = self.ser_pressure.readline()
            response = raw_response.decode('utf-8').strip()   # returns something like "*01 3.50E-03"
            value_str = response.split(' ', 1)[1]  # trim the "*01" (device ID)
            rough_valuetorr = float(value_str)     # "3.50E-03" -> 0.0035
        except Exception as e:
            log(f"Error occurred at Pressure: {e}")
            rough_valuetorr = "NA"

        try:
            self.ser_rfpower.reset_input_buffer() #drop leftover replies (e.g. to "***" or "S") so we read the answer to this Q
            self.ser_rfpower.write(COMMAND_RFPOWER.encode('utf-8'))
            raw_response = self.ser_rfpower.read_until(b'\r')   # R301 terminates with \r; readline() waits for \n until timeout
            response = raw_response.decode('utf-8').strip()   # returns something like "2320000 0 0 0 300"
            fields = response.split()           # [XXXXXXX, aaaa, bbbb, ccc, dddd]
            if len(fields) < 4:
                raise ValueError(f"unexpected response {raw_response!r}")
            forward_power_w = int(fields[2])
            reflected_power_w = int(fields[3])
        except Exception as e: #a bad reading is recorded as "NA"; the R301 is left running so a programmed routine isn't cut short
            log(f"Error occurred at RF: {e}")
            forward_power_w = "NA"
            reflected_power_w = "NA"

        return {
            "rough_pressure_pa": rough_valuetorr * TORR_TO_PA if rough_valuetorr != "NA" else "NA", #Pascal - ROUGH
            "fine_pressure_pa": (pressure_voltage_direct / 100) * TORR_TO_PA, #Pascal - FINE
            "voltage_v": voltage,
            "forward_power_w": forward_power_w,
            "reflected_power_w": reflected_power_w,
        }


class Acquisition(threading.Thread):
    #Runs the sampling loop off the GUI thread. Talks to the GUI only through queues, since tkinter isn't thread-safe:
    #  commands (GUI -> worker): ("record", (path, send_rf_go)), ("stop_record", None)
    #  events (worker -> GUI): ("log", msg), ("connected", None), ("sample", dict), ("recording", path or None),
    #                          ("camera_triggered", None), ("stopped", None)

    def __init__(self, events, trigger_camera):
        super().__init__(daemon=True)
        self.events = events
        self.trigger_camera = trigger_camera #send T at the first sample of the next recording (the camera is only triggered once per program run)
        self.instruments = Instruments()
        self.commands = queue.Queue()
        self.stop_event = threading.Event()
        self.csv_file = None
        self.csv_writer = None
        self.send_rf_go = False #send G at the start of the recording just opened
        self.rf_running = False #G was sent for the current recording, so stop the RF when it ends

    def log(self, msg):
        self.events.put(("log", msg))

    def run(self):
        instruments = self.instruments
        try:
            instruments.open(self.log)
            self.events.put(("connected", None))

            acquisition_start = now_s()
            i = 0
            camera_sent = False
            while not self.stop_event.is_set():
                if self.handle_commands(): #recording just started: restart the clock so the CSV starts at t=0
                    if self.trigger_camera:
                        camera_sent = instruments.trigger_camera(self.log) #sent right before t=0 so the video lines up with the first datapoint
                    if self.send_rf_go:
                        self.rf_running = instruments.start_rf_routine(self.log) #also right before t=0, so the routine starts with the data
                    acquisition_start = now_s()
                    i = 0
                    if self.rf_running:
                        self.log("RF routine started")
                elapsed_s = now_s() - acquisition_start

                sample = instruments.read(self.log)
                sample["time_s"] = elapsed_s

                if camera_sent: #check the trigger's reply now that the first sample is taken
                    camera_sent = False
                    self.trigger_camera = False
                    reply = instruments.camera_reply()
                    if reply != "OK":
                        self.log(f"WARNING: Unexpected reply from camera trigger: {reply!r}")
                    self.events.put(("camera_triggered", None))

                if self.csv_writer is not None:
                    self.csv_writer.writerow([f"{elapsed_s:.4f}"] + [sample[key] for key in CSV_HEADER[1:]])
                    self.csv_file.flush() #ensure data survives a crash/interrupt
                self.events.put(("sample", sample))

                i += 1
                sleep_until(acquisition_start + i * SAMPLE_INTERVAL_S, self.stop_event)
        except Exception as e:
            self.log(f"WARNING: Unable to read from devices, acquisition stopped. Check connection to devices. Code: {e}")
        finally:
            self.end_recording()
            instruments.close()
            self.events.put(("stopped", None))

    def handle_commands(self):
        #returns True if a new recording was started
        started = False
        while True:
            try:
                command, arg = self.commands.get_nowait()
            except queue.Empty:
                return started

            if command == "record":
                path, self.send_rf_go = arg
                self.close_csv()
                try:
                    self.csv_file = open(path, "w", newline="")
                except OSError as e:
                    self.log(f"Could not open {path}: {e}")
                    self.events.put(("recording", None))
                    continue
                self.csv_writer = csv.writer(self.csv_file)
                self.csv_writer.writerow(CSV_HEADER)
                started = True
                self.events.put(("recording", path))
            elif command == "stop_record":
                self.end_recording()
                self.events.put(("recording", None))

    def end_recording(self):
        #close the CSV, re-arm the camera trigger and stop the RF routine, if a recording was running
        if self.csv_file is None:
            return
        self.close_csv()
        self.instruments.reset_camera(self.log)
        if self.rf_running:
            self.rf_running = False
            self.instruments.stop_rf_routine(self.log)

    def close_csv(self):
        if self.csv_file is not None:
            quietly(self.csv_file.close)
        self.csv_file = None
        self.csv_writer = None


class App:
    def __init__(self, root):
        self.root = root
        self.events = queue.Queue()
        self.acquisition = None
        self.recording_path = None
        self.rows_recorded = 0
        self.auto_name = default_file_name()
        self.camera_triggered = False

        root.title("Voltage / Pressure Data Collector")
        root.protocol("WM_DELETE_WINDOW", self.on_close)
        main = ttk.Frame(root, padding=10)
        main.grid(sticky="nsew")
        root.columnconfigure(0, weight=1)
        root.rowconfigure(0, weight=1)
        main.columnconfigure(0, weight=1)
        main.rowconfigure(3, weight=1)

        #save location
        save_frame = ttk.LabelFrame(main, text="Save location", padding=8)
        save_frame.grid(row=0, column=0, sticky="ew")
        save_frame.columnconfigure(1, weight=1)
        self.dir_var = tk.StringVar(value=DEFAULT_DIR)
        self.name_var = tk.StringVar(value=self.auto_name)
        ttk.Label(save_frame, text="Directory:").grid(row=0, column=0, sticky="w")
        self.dir_entry = ttk.Entry(save_frame, textvariable=self.dir_var)
        self.dir_entry.grid(row=0, column=1, sticky="ew", padx=5)
        self.browse_button = ttk.Button(save_frame, text="Browse…", command=self.browse_dir)
        self.browse_button.grid(row=0, column=2)
        ttk.Label(save_frame, text="File name:").grid(row=1, column=0, sticky="w", pady=(5, 0))
        self.name_entry = ttk.Entry(save_frame, textvariable=self.name_var)
        self.name_entry.grid(row=1, column=1, sticky="ew", padx=5, pady=(5, 0))
        ttk.Label(save_frame, text=".csv").grid(row=1, column=2, sticky="w", pady=(5, 0))

        #controls
        control_frame = ttk.Frame(main, padding=(0, 10))
        control_frame.grid(row=1, column=0, sticky="ew")
        self.display_button = ttk.Button(control_frame, text="Start display", width=18, command=self.toggle_display)
        self.display_button.grid(row=0, column=0, padx=(0, 10))
        self.record_button = ttk.Button(control_frame, text="Start recording", width=18, command=self.toggle_recording)
        self.record_button.grid(row=0, column=1, padx=(0, 10))
        self.rf_go_var = tk.BooleanVar(value=False) #send G to the R301 when recording starts
        self.rf_go_button = ttk.Checkbutton(control_frame, variable=self.rf_go_var, style="Toolbutton", width=18, command=self.update_rf_go_button)
        self.rf_go_button.grid(row=0, column=2, padx=(0, 10))
        self.update_rf_go_button()
        self.status_var = tk.StringVar(value="Idle")
        ttk.Label(control_frame, textvariable=self.status_var).grid(row=0, column=3, sticky="w")

        #live data boxes
        data_frame = ttk.Frame(main)
        data_frame.grid(row=2, column=0, sticky="ew")
        self.value_vars = {}
        for n, (key, label, unit, _) in enumerate(PARAMETERS):
            box = ttk.LabelFrame(data_frame, text=f"{label} ({unit})", padding=8)
            box.grid(row=n // 3, column=n % 3, sticky="nsew", padx=4, pady=4)
            data_frame.columnconfigure(n % 3, weight=1, uniform="box")
            self.value_vars[key] = tk.StringVar(value="—")
            ttk.Label(box, textvariable=self.value_vars[key], font=("Courier New", 20, "bold"), anchor="e", width=11).pack(fill="x")

        #error/status log
        log_frame = ttk.LabelFrame(main, text="Log", padding=4)
        log_frame.grid(row=3, column=0, sticky="nsew", pady=(10, 0))
        self.log_text = ScrolledText(log_frame, height=8, state="disabled", wrap="word")
        self.log_text.pack(fill="both", expand=True)

        self.root.after(100, self.poll_events)

    #---- button handlers ----

    def browse_dir(self):
        chosen = filedialog.askdirectory(initialdir=self.dir_var.get() or ".")
        if chosen:
            self.dir_var.set(chosen)

    def update_rf_go_button(self):
        self.rf_go_button.config(text="RF routine: ON" if self.rf_go_var.get() else "RF routine: OFF")

    def toggle_display(self):
        if self.acquisition is None:
            self.start_acquisition()
        else:
            self.acquisition.stop_event.set()
            self.display_button.config(state="disabled")
            self.record_button.config(state="disabled")
            self.status_var.set("Stopping…")

    def toggle_recording(self):
        if self.recording_path is not None:
            self.acquisition.commands.put(("stop_record", None))
            self.record_button.config(state="disabled")
            return

        output_file = self.output_file()
        if output_file is None:
            return
        record = (output_file, self.rf_go_var.get())
        if self.acquisition is None: #recording needs live sampling, so start it too
            self.start_acquisition(record=record)
        else:
            self.acquisition.commands.put(("record", record))
        self.record_button.config(state="disabled")
        self.set_save_inputs("disabled")

    def output_file(self):
        output_dir = pathlib.Path(self.dir_var.get().strip() or DEFAULT_DIR)
        name = pathlib.Path(self.name_var.get().strip() or default_file_name()).stem
        try:
            output_dir.mkdir(parents=True, exist_ok=True)
        except OSError as e:
            messagebox.showerror("Save location", f"Could not create {output_dir}:\n{e}")
            return None
        output_file = output_dir / f"{name}.csv"
        if output_file.exists() and not messagebox.askyesno("Overwrite?", f"{output_file} already exists. Overwrite it?"):
            return None
        return output_file

    def start_acquisition(self, record=None):
        self.acquisition = Acquisition(self.events, trigger_camera=not self.camera_triggered)
        if record is not None: #queue before the thread starts so the very first sample is recorded (and triggers the camera)
            self.acquisition.commands.put(("record", record))
        self.acquisition.start()
        self.display_button.config(text="Stop display")
        self.status_var.set("Connecting…")

    def on_close(self):
        if self.acquisition is not None:
            if self.recording_path is not None and not messagebox.askyesno("Quit", "Recording is in progress. Stop and quit?"):
                return
            self.acquisition.stop_event.set()
            self.acquisition.join(timeout=5) #let it close the CSV and instruments
        self.root.destroy()

    #---- worker events ----

    def poll_events(self):
        while True:
            try:
                event, arg = self.events.get_nowait()
            except queue.Empty:
                break
            if event == "sample":
                self.show_sample(arg)
            elif event == "log":
                self.log(arg)
            elif event == "connected":
                self.status_var.set("Displaying")
                self.log("Instruments connected")
            elif event == "recording":
                self.on_recording_changed(arg)
            elif event == "camera_triggered":
                self.camera_triggered = True
                self.log("Camera triggered")
            elif event == "stopped":
                self.on_stopped()
        self.root.after(100, self.poll_events)

    def show_sample(self, sample):
        for key, _, _, fmt in PARAMETERS:
            value = sample[key]
            self.value_vars[key].set(value if value == "NA" else fmt.format(value))
        if self.recording_path is not None:
            self.rows_recorded += 1
            self.status_var.set(f"Recording to {self.recording_path.name} ({self.rows_recorded} samples)")

    def on_recording_changed(self, path):
        self.record_button.config(state="normal")
        if path is not None:
            self.recording_path = path
            self.rows_recorded = 0
            self.record_button.config(text="Stop recording")
            self.status_var.set(f"Recording to {path.name}")
            self.log(f"Recording to {path}")
            return
        if self.recording_path is not None:
            self.log(f"Saved {self.rows_recorded} samples to {self.recording_path}")
        self.recording_path = None
        self.record_button.config(text="Start recording")
        self.set_save_inputs("normal")
        if self.acquisition is not None:
            self.status_var.set("Displaying")
        if self.name_var.get() == self.auto_name: #fresh default name so the next recording doesn't overwrite this one
            self.auto_name = default_file_name()
            self.name_var.set(self.auto_name)

    def on_stopped(self):
        self.acquisition = None
        self.on_recording_changed(None) #worker closes any open CSV on stop
        self.display_button.config(text="Start display", state="normal")
        self.status_var.set("Idle")
        self.log("Acquisition stopped")

    def set_save_inputs(self, state):
        for widget in (self.dir_entry, self.name_entry, self.browse_button, self.rf_go_button): #locked while recording; they only apply at the start
            widget.config(state=state)

    def log(self, msg):
        self.log_text.config(state="normal")
        self.log_text.insert("end", msg + "\n")
        self.log_text.see("end")
        self.log_text.config(state="disabled")


if __name__ == "__main__":
    root = tk.Tk()
    App(root)
    root.mainloop()
