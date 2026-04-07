#!/usr/bin/env python3

import os
import subprocess
import zipfile
import urllib.request
import json
import time
import argparse
from pathlib import Path

# Configuration
CACHE_DIR = Path("cache")
MAPS_ZIP_URL = "https://www.movingai.com/benchmarks/mapf/mapf-map.zip"
SCEN_ZIP_URL = "https://www.movingai.com/benchmarks/mapf/mapf-scen-random.zip"
DEFAULT_AGENT_COUNTS = [2, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50]

def download_file(url, dest):
    if dest.exists():
        return
    print(f"Downloading {url} to {dest}...")
    urllib.request.urlretrieve(url, dest)

def unzip_file(zip_path, extract_to):
    print(f"Unzipping {zip_path} to {extract_to}...")
    with zipfile.ZipFile(zip_path, 'r') as zip_ref:
        zip_ref.extractall(extract_to)

def run_benchmark(map_path, scen_path, num_agents, timeout):
    cmd = [
        "cargo", "run", "-p", "mapf-bench", "--release", "--",
        "--map", str(map_path),
        "--scen", str(scen_path),
        "--num-agents", str(num_agents),
        "--timeout", str(timeout)
    ]
    
    start_time = time.time()
    try:
        # Strict timeout enforcement via subprocess
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout + 5)
        duration = time.time() - start_time
        
        success = "Negotiation successful" in result.stdout
        return {
            "success": success,
            "duration": duration,
            "stdout": result.stdout,
            "stderr": result.stderr
        }
    except subprocess.TimeoutExpired:
        return {
            "success": False,
            "duration": timeout,
            "error": "Timeout"
        }
    except Exception as e:
        return {
            "success": False,
            "error": str(e)
        }

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=int, default=30, help="Timeout in seconds per run")
    parser.add_argument("--max-scenarios", type=int, default=1, help="Max random scenarios per map")
    parser.add_argument("--maps", nargs="+", default=["empty-32-32.map", "room-32-32-4.map", "maze-32-32-2.map"])
    args = parser.parse_args()

    CACHE_DIR.mkdir(exist_ok=True)
    
    maps_zip = CACHE_DIR / "mapf-map.zip"
    scen_zip = CACHE_DIR / "mapf-scen-random.zip"
    
    download_file(MAPS_ZIP_URL, maps_zip)
    download_file(SCEN_ZIP_URL, scen_zip)
    
    maps_dir = CACHE_DIR / "maps"
    scen_dir = CACHE_DIR / "scenarios"
    
    if not maps_dir.exists():
        unzip_file(maps_zip, maps_dir)
    if not scen_dir.exists():
        unzip_file(scen_zip, scen_dir)
    
    report = {}

    print("Building mapf-bench in release mode...")
    subprocess.run(["cargo", "build", "-p", "mapf-bench", "--release"], check=True)

    for map_name in args.maps:
        map_path = maps_dir / map_name
        if not map_path.exists():
            print(f"Map {map_name} not found.")
            continue
            
        base_name = map_name.replace(".map", "")
        
        # Find all matching scenario files
        scen_pattern = f"{base_name}-random-*.scen"
        scen_files = list((scen_dir / "scen-random").glob(scen_pattern))
        scen_files.sort()
        
        if not scen_files:
            print(f"No scenarios found for {map_name} with pattern {scen_pattern}")
            continue

        selected_scens = scen_files[:args.max_scenarios]
        
        for scen_path in selected_scens:
            scen_name = scen_path.name
            print(f"\nBenchmarking {map_name} with scenario {scen_name}...")
            
            key = f"{map_name}:{scen_name}"
            report[key] = []
            
            for count in DEFAULT_AGENT_COUNTS:
                print(f"  Agents: {count}", end=" ", flush=True)
                res = run_benchmark(map_path, scen_path, count, args.timeout)
                if res["success"]:
                    print(f"✅ ({res['duration']:.2f}s)")
                else:
                    error_msg = res.get("error", "FAILED")
                    print(f"❌ ({error_msg})")
                
                report[key].append({
                    "agents": count,
                    "success": res["success"],
                    "duration": res.get("duration", 0),
                    "error": res.get("error")
                })

    # Save report
    with open("benchmark_report.json", "w") as f:
        json.dump(report, f, indent=2)
    
    # Print summary table
    print("\nBenchmark Summary:")
    print(f"{'Scenario':<40} | {'Agents':<6} | {'Status':<8} | {'Time':<8}")
    print("-" * 75)
    for key, results in report.items():
        for res in results:
            status = "SUCCESS" if res["success"] else (res.get("error") if res.get("error") else "FAILED")
            print(f"{key[:40]:<40} | {res['agents']:<6} | {status:<8} | {res['duration']:>7.2f}s")

if __name__ == "__main__":
    main()
