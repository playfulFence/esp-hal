use std::{collections::HashMap, fs, path::PathBuf, process::Command};

use anyhow::Result;
use minijinja::{context, Environment, Source};
use serde::{Deserialize, Serialize};
use strum::{Display, EnumIter};

use crate::Chip;

#[derive(Debug, Clone, Copy, Display, EnumIter, Deserialize, Serialize)]
pub enum Runtime {
    #[serde(rename = "riscv_rt")]
    #[strum(serialize = "riscv_rt")]
    RiscV,
    #[serde(rename = "xtensa_lx_rt")]
    #[strum(serialize = "xtensa_lx_rt")]
    Xtensa,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct Example {
    pub name: String,
    pub extra: HashMap<String, String>,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct Metadata {
    pub chip: Chip,
    pub rt: Runtime,
    pub setup: String,
    pub examples: Vec<Example>,
}

impl Metadata {
    pub fn load(workspace: &PathBuf, chip: Chip) -> Result<Self> {
        let path = workspace.join("xtask").join("resources").join("metadata");
        let path = path.join(format!("{chip}.toml"));
        let meta = fs::read_to_string(path)?;

        let meta: Self = toml::from_str(&meta)?;

        Ok(meta)
    }
}

pub fn generate_examples(workspace: &PathBuf, chips: Vec<Chip>) -> Result<()> {
    let templates = workspace.join("xtask").join("resources").join("templates");

    for chip in chips {
        log::info!("Chip: {chip}");
        let examples = workspace.join(format!("{chip}-hal")).join("examples");

        let meta = Metadata::load(&workspace, chip)?;
        let env = init_environment(&templates, &meta)?;

        for Example { name, extra } in &meta.examples {
            let tmpl = env.get_template(&format!("{name}.tmpl"))?;
            log::info!("-> Generating: {}", name);

            let ctx = context! {
                chip  => meta.chip.to_string(),
                rt    => meta.rt.to_string(),
                setup => meta.setup,
                extra => extra,
            };

            let example = examples.join(format!("{name}.rs"));
            let text = tmpl.render(ctx)?;

            fs::write(&example, text)?;
            Command::new("rustfmt").arg(example).output()?;
        }
    }

    Ok(())
}

fn init_environment<'env>(
    templates: &'env PathBuf,
    meta: &'env Metadata,
) -> Result<Environment<'env>> {
    let mut source = Source::from_path(&templates);
    for Example { name, extra: _ } in &meta.examples {
        let tmpl = templates.join(format!("{name}.tmpl"));
        let tmpl = fs::read_to_string(&tmpl)?;

        source.add_template(name, tmpl)?;
    }

    let mut env = Environment::new();
    env.set_source(source);

    Ok(env)
}
