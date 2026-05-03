//! IGE All-in-One Runner
//!
//! Runs all tests, benchmarks, and generates visualization.
//!
//! Usage:
//!   cargo run --package ige-core --example run_all
//!   cargo run --package ige-core --example run_all -- --skip-tests --skip-bench
//!
//! Individual modes:
//!   cargo run --package ige-core --example run_all -- --tests-only
//!   cargo run --package ige-core --example run_all -- --bench-only

use std::process::Command;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let skip_tests = args.contains(&"--skip-tests".to_string());
    let skip_bench = args.contains(&"--skip-bench".to_string());
    let tests_only = args.contains(&"--tests-only".to_string());
    let bench_only = args.contains(&"--bench-only".to_string());

    println!("═══════════════════════════════════════════════════════════");
    println!("  IGE - Inscribed Geometry Engine - Run All");
    println!("═══════════════════════════════════════════════════════════\n");

    // Run tests
    if !skip_bench && !tests_only {
        if !skip_tests {
            println!("\n[1/3] Running tests...\n");
            let status = Command::new("cargo")
                .args(["test", "--package", "ige-core", "--", "--nocapture"])
                .status()
                .expect("Failed to run tests");
            
            if !status.success() {
                eprintln!("Tests failed!");
                std::process::exit(1);
            }
        }

        // Run benches
        println!("\n[2/3] Running benchmarks...\n");
        let status = Command::new("cargo")
            .args(["bench", "--package", "ige-core", "--", "--noplot", "--sample-size", "10"])
            .status()
            .expect("Failed to run benches");
        
        if !status.success() {
            eprintln!("Benches failed!");
            std::process::exit(1);
        }
    } else if tests_only {
        println!("\n[1/1] Running tests only...\n");
        let status = Command::new("cargo")
            .args(["test", "--package", "ige-core", "--", "--nocapture"])
            .status()
            .expect("Failed to run tests");
        
        if !status.success() {
            eprintln!("Tests failed!");
            std::process::exit(1);
        }
    }

    if !tests_only && !bench_only {
        // Run visualization
        println!("\n[3/3] Generating visualization...\n");
        let status = Command::new("cargo")
            .args(["run", "--package", "ige-core", "--example", "visualize", "--", "--limit", "10"])
            .status()
            .expect("Failed to run visualize");
        
        if !status.success() {
            eprintln!("Visualization failed!");
            std::process::exit(1);
        }
        
        println!("\n✓ Visualization saved to target/ige_output/index.html");
    }

    println!("\n═══════════════════════════════════════════════════════════");
    println!("  All tasks completed successfully!");
    println!("═══════════════════════════════════════════════════════════");
}