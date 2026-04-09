import json
with open("benchmark_report_main.json") as f:
    report = json.load(f)

with open("main_branch_benchmark.md", "w") as f:
    f.write("# Benchmark Report: Main Branch (Baseline)\n\n")
    f.write("| Scenario | Agents | Status | Time |\n")
    f.write("| --- | --- | --- | --- |\n")
    for key, results in report.items():
        for res in results:
            status = "SUCCESS" if res["success"] else (res.get("error") if res.get("error") else "FAILED")
            f.write(f"| {key} | {res['agents']} | {status} | {res['duration']:>7.2f}s |\n")
