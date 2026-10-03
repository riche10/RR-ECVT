"""Check source preservation and run both firmware variants against mocked I/O."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess

parser = argparse.ArgumentParser()
parser.add_argument("--cxx", required=True, help="Path to a host C++ compiler")
parser.add_argument("--output-dir", required=True)
args = parser.parse_args()
root = Path(__file__).resolve().parents[2]
tests = Path(__file__).resolve().parent
out = Path(args.output_dir).resolve()
out.mkdir(parents=True, exist_ok=True)

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

bench = root / "Bench Testing"
auto = root / "Automatic"
assert sha(bench / "src/main.cpp") == "292a12c93a2d8e3c45c48edd19d6c4e872d34d6ab0051769a3ba32ee7dc102d3", "Bench source changed"
for rel in ["platformio.ini", "include/MyCANBUS.h"]:
    assert sha(bench / rel) == sha(auto / rel), f"Unexpected dependency/config change: {rel}"
for file in (bench / "lib").rglob("*"):
    if file.is_file():
        assert sha(file) == sha(auto / file.relative_to(bench)), f"Library differs: {file}"

bench_source = (bench / "src/main.cpp").read_text()
auto_source = (auto / "src/main.cpp").read_text()
for removed in ["TerminalState", "automaticControlArmed", "pulseMotor", "handleSerialCommands",
                "manualPulseDurationMs", "forceSend", "Serial.read", "Serial.available"]:
    assert removed not in auto_source, f"Bench control remains: {removed}"

def function(source, name):
    # Locate a complete definition, skipping forward declarations.
    match = re.search(r"^\w+\s+" + re.escape(name) + r"\s*\([^;{}]*\)\s*\{", source, re.M)
    assert match, f"Missing function: {name}"
    start = match.start()
    position = match.end()
    depth = 1
    while depth:
        if source[position] == "{": depth += 1
        if source[position] == "}": depth -= 1
        position += 1
    return source[start:position]

unchanged = ["setupCan", "setupInputs", "setupMotorOutput", "setupInterrupts", "readSensors",
             "readSheavePosition", "calculatePotPercent", "capturePrimaryInterval",
             "captureRearWheelInterval", "updateRpmFromInterval", "combineAverage",
             "applyRpmTimeouts", "clearRpmState", "calculateControl", "chooseActiveTarget",
             "calculateMotorPwm", "applySafetyClamp", "receiveCanData", "forceNeutral",
             "PR_RPM_ISR", "RW_RPM_ISR"]
for name in unchanged:
    assert function(bench_source, name) == function(auto_source, name), f"Core function changed: {name}"
def config_block(source):
    return re.search(r"struct TcuConfig \{.*?^\};", source, re.M | re.S)[0]
assert config_block(bench_source).replace("    uint32_t manualPulseDurationMs = 100;\n", "") == config_block(auto_source)

results = []
compiler_env = os.environ.copy()
compiler_env["PATH"] = str(Path(args.cxx).resolve().parent) + os.pathsep + compiler_env.get("PATH", "")
for name, folder, expected in [("bench", bench, 0), ("automatic", auto, 1)]:
    binary = out / (name + "-test.exe")
    compile_command = [args.cxx, "-std=c++17", "-Wall", "-Wextra",
                       f"-DEXPECT_AUTOMATIC={expected}", "-I", str(tests / "stubs"),
                       "-I", str(folder / "src"), str(tests / "variant_test.cpp"),
                       "-o", str(binary), "-static"]
    compilation = subprocess.run(compile_command, text=True, capture_output=True, timeout=45, env=compiler_env)
    if compilation.returncode:
        raise RuntimeError(f"Compiler exit {compilation.returncode}: " + compilation.stdout + compilation.stderr)
    execution = subprocess.run([str(binary)], text=True, capture_output=True, check=True, timeout=10)
    print(name + ": " + execution.stdout.strip())
    results.append({"variant": name, "compile_warnings": compilation.stderr,
                    "result": execution.stdout, "source_sha256": sha(folder / "src/main.cpp")})
fingerprints = [re.search(r"CONTROL_GRID_FINGERPRINT=(\d+)", r["result"])[1] for r in results]
assert len(set(fingerprints)) == 1, "Control calculations differ between variants"
report = {"source_preservation": "PASS", "automatic_bench_controls_removed": "PASS",
          "unchanged_core_functions": unchanged,
          "control_grid_cases_per_variant": 120, "control_grid_equal": True,
          "hardware_tested": False, "results": results}
(out / "verification-results.json").write_text(json.dumps(report, indent=2) + "\n")
print("PASS: bench preserved; automatic has no bench controls; 21 core functions and 120 control cases identical")
