use std::path::PathBuf;

use anyhow::Result;
use clap::{Parser, Subcommand};
use log::LevelFilter;
use serde::{Deserialize, Serialize};
use strum::{Display, EnumIter, EnumString, IntoEnumIterator};

use self::generate::generate_examples;

mod generate;

#[derive(Debug, Clone, Copy, Display, EnumIter, EnumString, Deserialize, Serialize)]
#[serde(rename_all = "lowercase")]
#[strum(serialize_all = "lowercase")]
pub enum Chip {
    Esp32,
    Esp32c3,
    Esp32s2,
    Esp32s3,
}

#[derive(Debug, Parser)]
struct Cli {
    #[command(subcommand)]
    subcommand: Commands,
}

#[derive(Debug, Subcommand)]
enum Commands {
    /// Generate all examples
    Generate {
        /// (Optional) one or more chips to generate examples for
        chips: Vec<Chip>,
    },
}

fn main() -> Result<()> {
    env_logger::Builder::new()
        .filter_module("xtask", LevelFilter::Info)
        .init();

    let cli = Cli::parse();

    let workspace = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let workspace = workspace.parent().unwrap().canonicalize()?;

    let Commands::Generate { chips } = cli.subcommand;
    let chips = if chips.is_empty() {
        Chip::iter().collect()
    } else {
        chips
    };

    log::info!(
        "Generating examples for: {}",
        chips
            .iter()
            .map(|c| c.to_string())
            .collect::<Vec<_>>()
            .join(", ")
    );
    generate_examples(&workspace, chips)?;

    Ok(())
}
