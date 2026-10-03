"""Package verified ECVT sources and artifacts without including build caches."""
import argparse
import difflib
import hashlib
import json
from pathlib import Path
import shutil
import zipfile

parser = argparse.ArgumentParser()
parser.add_argument("--original", type=Path, required=True)
parser.add_argument("--test-results", type=Path, required=True)
parser.add_argument("--output-zip", type=Path, required=True)
args = parser.parse_args()
root = Path(__file__).resolve().parent.parent
development = root / "Development"

def digest(data):
    return hashlib.sha256(data).hexdigest()

original_files = [p for p in args.original.rglob("*") if p.is_file()]
snapshot = development / "Original RR ECVT CODE 2.0.zip"
with zipfile.ZipFile(snapshot) as archive:
    names = {info.filename for info in archive.infolist() if not info.is_dir()}
    expected = {args.original.name + "/" + p.relative_to(args.original).as_posix() for p in original_files}
    assert names == expected, "Snapshot contents differ from original folder"
    for source in original_files:
        key = args.original.name + "/" + source.relative_to(args.original).as_posix()
        assert digest(source.read_bytes()) == digest(archive.read(key)), f"Snapshot differs: {key}"

for source in original_files:
    rel = source.relative_to(args.original)
    if rel.parts[0] in {"src", "include", "lib", "platformio.ini"}:
        assert digest(source.read_bytes()) == digest((root / "Bench Testing" / rel).read_bytes()), \
            f"Bench preservation failed: {rel}"

for variant, stem in [("Bench Testing", "Bench-Testing"), ("Automatic", "Automatic")]:
    build_dir = root / variant / ".pio/build/teensy40"
    destination = root / variant / "Compiled"
    destination.mkdir(exist_ok=True)
    for ext in ["hex", "elf"]:
        shutil.copy2(build_dir / ("firmware." + ext), destination / (stem + "." + ext))
    log = (development / (stem + "-build.log")).read_text(encoding="utf-8-sig")
    assert "[SUCCESS]" in log, f"Build log not successful: {variant}"

shutil.copy2(args.test_results, development / "verification-results.json")
bench_source = (root / "Bench Testing/src/main.cpp").read_text().splitlines(keepends=True)
auto_source = (root / "Automatic/src/main.cpp").read_text().splitlines(keepends=True)
(development / "variant-diff.patch").write_text("".join(difflib.unified_diff(
    bench_source, auto_source, fromfile="Bench Testing/src/main.cpp", tofile="Automatic/src/main.cpp")))

excluded_parts = {".pio", ".git", "__pycache__"}
manifest_path = development / "SHA256SUMS.json"
files = sorted(p for p in root.rglob("*") if p.is_file() and not (set(p.relative_to(root).parts) & excluded_parts))
manifest = {
    "date": "2026-10-02",
    "original_snapshot_files_verified": len(original_files),
    "bench_source_preserved": True,
    "hardware_tested": False,
    "files": [{"path": p.relative_to(root).as_posix(), "bytes": p.stat().st_size,
               "sha256": digest(p.read_bytes())} for p in files if p != manifest_path],
}
manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
files = sorted(set(files) | {manifest_path})
with zipfile.ZipFile(args.output_zip, "w", zipfile.ZIP_DEFLATED) as archive:
    for path in files:
        archive.write(path, "ECVT/" + path.relative_to(root).as_posix())
with zipfile.ZipFile(args.output_zip) as archive:
    assert archive.testzip() is None
    for file in manifest["files"]:
        assert digest(archive.read("ECVT/" + file["path"])) == file["sha256"]
print(json.dumps({"archive": str(args.output_zip.resolve()), "files": len(files),
                  "bytes": args.output_zip.stat().st_size,
                  "sha256": digest(args.output_zip.read_bytes()),
                  "snapshot_files_verified": len(original_files)}, indent=2))
