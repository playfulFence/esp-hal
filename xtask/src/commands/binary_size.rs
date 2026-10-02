use std::{
    path::{Path, PathBuf},
    process::Command,
};

use anyhow::{Context, Result, bail, ensure};
use clap::{Args, Subcommand};

use super::build_examples;
use crate::{
    Package,
    binary_size::{
        self,
        ChipSizes,
        Commit,
        Config,
        Culprit,
        Regression,
        Sizes,
        first_parent_commits,
        first_regressed,
    },
    firmware::Metadata,
    metadata::Chip,
    windows_safe_path,
};

/// Binary size tracking of the examples.
#[derive(Debug, Subcommand)]
pub enum BinarySizeCmds {
    /// Build the configured examples for one chip, and write their sizes to a JSON file.
    Measure(MeasureArgs),
    /// Compare the sizes of two commits, and write a markdown report and the list of regressions.
    Compare(CompareArgs),
    /// Find the commit that caused the worst regression.
    Bisect(BisectArgs),
    /// Write the body of an issue that reports regressions.
    Issue(IssueArgs),
}

#[derive(Debug, Args)]
pub struct MeasureArgs {
    /// Chip to build the examples for.
    #[arg(value_enum)]
    chip: Chip,

    /// Which examples to measure and their thresholds.
    #[arg(long, default_value = ".github/binary-size.json")]
    config: PathBuf,

    /// The JSON file to write.
    #[arg(long)]
    output: PathBuf,

    /// The toolchain to build with.
    #[arg(long)]
    toolchain: Option<String>,
}

#[derive(Debug, Args)]
pub struct CompareArgs {
    /// Directory with the JSON files of the base commit.
    #[arg(long)]
    base: PathBuf,

    /// Directory with the JSON files of the head commit.
    #[arg(long)]
    head: PathBuf,

    /// Which examples to measure and their thresholds.
    #[arg(long, default_value = ".github/binary-size.json")]
    config: PathBuf,

    /// The markdown report to write.
    #[arg(long)]
    report: PathBuf,

    /// The JSON list of regressions to write, worst first.
    #[arg(long)]
    regressions: PathBuf,
}

#[derive(Debug, Args)]
pub struct BisectArgs {
    /// The JSON list of regressions written by `compare`. The first one is bisected.
    #[arg(long)]
    regressions: PathBuf,

    /// The commit the regressions were measured against.
    #[arg(long)]
    base: String,

    /// The commit that has the regressions.
    #[arg(long)]
    head: String,

    /// Which examples to measure and their thresholds.
    #[arg(long, default_value = ".github/binary-size.json")]
    config: PathBuf,

    /// The JSON file to write the culprit to.
    #[arg(long)]
    output: PathBuf,

    /// Give up if the range has more first-parent commits than this.
    #[arg(long, default_value_t = 32)]
    max_commits: usize,

    /// The toolchain to build with.
    #[arg(long)]
    toolchain: Option<String>,
}

#[derive(Debug, Args)]
pub struct IssueArgs {
    /// The JSON list of regressions written by `compare`.
    #[arg(long)]
    regressions: PathBuf,

    /// The markdown report written by `compare`.
    #[arg(long)]
    report: PathBuf,

    /// The JSON file written by `bisect`. Ignored if it does not exist.
    #[arg(long)]
    culprit: Option<PathBuf>,

    /// The commit the regressions were measured against.
    #[arg(long)]
    base: String,

    /// The commit that has the regressions.
    #[arg(long)]
    head: String,

    /// Link to the workflow run that found the regressions.
    #[arg(long)]
    run_url: Option<String>,

    /// The markdown file to write.
    #[arg(long)]
    output: PathBuf,
}

pub fn binary_size(workspace: &Path, command: BinarySizeCmds) -> Result<()> {
    match command {
        BinarySizeCmds::Measure(args) => measure(workspace, args),
        BinarySizeCmds::Compare(args) => compare(args),
        BinarySizeCmds::Bisect(args) => bisect(workspace, args),
        BinarySizeCmds::Issue(args) => issue(workspace, args),
    }
}

fn measure(workspace: &Path, args: MeasureArgs) -> Result<()> {
    let config = Config::load(&args.config)?;
    let examples = crate::firmware::load_package(workspace, Package::Examples)?;

    let mut sizes = ChipSizes {
        chip: args.chip,
        commit: git(workspace, &["rev-parse", "HEAD"])?,
        rustc: rustc_version(workspace, args.chip, args.toolchain.as_deref())?,
        examples: Default::default(),
    };
    for name in &config.examples {
        match build_and_measure(
            workspace,
            &examples,
            args.chip,
            name,
            args.toolchain.as_deref(),
        )? {
            Some(measured) => {
                sizes.examples.insert(name.clone(), measured);
            }
            None => log::info!("`{name}` does not support {}, skipping it", args.chip),
        }
    }

    write_json(&args.output, &sizes)
}

fn compare(args: CompareArgs) -> Result<()> {
    let config = Config::load(&args.config)?;
    let base = ChipSizes::load_dir(&args.base)?;
    let head = ChipSizes::load_dir(&args.head)?;
    ensure!(
        !head.is_empty(),
        "{} contains no sizes",
        args.head.display()
    );

    let comparison = binary_size::compare(&base, &head, &config);

    write_file(&args.report, &comparison.report)?;
    write_json(&args.regressions, &comparison.regressions)
}

fn bisect(workspace: &Path, args: BisectArgs) -> Result<()> {
    let config = Config::load(&args.config)?;
    let regressions: Vec<Regression> = read_json(&args.regressions)?;
    let Some(regression) = regressions.into_iter().next() else {
        bail!("There is no regression to bisect");
    };
    let threshold = config.thresholds[&regression.region];

    let commits = first_parent_commits(workspace, &args.base, &args.head)?;
    ensure!(
        commits.len() <= args.max_commits,
        "{}..{} has {} commits, more than --max-commits {}",
        args.base,
        args.head,
        commits.len(),
        args.max_commits
    );
    ensure!(
        git(
            workspace,
            &["status", "--porcelain", "--untracked-files=no"]
        )?
        .is_empty(),
        "Bisecting checks out other commits. Commit or stash your changes first."
    );

    // The branch, or the commit when nothing is checked out.
    let original = match git(workspace, &["symbolic-ref", "--quiet", "--short", "HEAD"]) {
        Ok(branch) => branch,
        Err(_) => git(workspace, &["rev-parse", "HEAD"])?,
    };
    let result = first_regressed(commits.len(), |index| {
        let commit = &commits[index];
        log::info!("Measuring {} at {}", regression.example, commit.sha);
        git(workspace, &["checkout", "--quiet", "--detach", &commit.sha])?;

        let examples = crate::firmware::load_package(workspace, Package::Examples)?;
        let sizes = build_and_measure(
            workspace,
            &examples,
            regression.chip,
            &regression.example,
            args.toolchain.as_deref(),
        )?
        .with_context(|| {
            format!(
                "`{}` does not support {} at {}",
                regression.example, regression.chip, commit.sha
            )
        })?;

        Ok(threshold.exceeded(regression.base, sizes.get(regression.region)))
    });
    git(workspace, &["checkout", "--quiet", &original])?;

    let commit: Commit = commits[result?].clone();
    log::info!("Culprit: {} {}", commit.sha, commit.subject);

    write_json(&args.output, &Culprit { regression, commit })
}

fn issue(workspace: &Path, args: IssueArgs) -> Result<()> {
    let regressions: Vec<Regression> = read_json(&args.regressions)?;
    let report = std::fs::read_to_string(&args.report)
        .with_context(|| format!("Failed to read {}", args.report.display()))?;
    let culprit: Option<Culprit> = match args.culprit {
        Some(path) if path.exists() => Some(read_json(&path)?),
        _ => None,
    };
    let commits = first_parent_commits(workspace, &args.base, &args.head)?;

    let body = binary_size::issue_body(
        &regressions,
        &commits,
        culprit.as_ref(),
        &report,
        args.run_url.as_deref(),
    );

    write_file(&args.output, &body)
}

/// Builds `name` for `chip` in release mode and measures it. Returns `None` if the example does
/// not support the chip.
fn build_and_measure(
    workspace: &Path,
    examples: &[Metadata],
    chip: Chip,
    name: &str,
    toolchain: Option<&str>,
) -> Result<Option<Sizes>> {
    ensure!(
        examples.iter().any(|example| example.matches_name(name)),
        "There is no example called `{name}`"
    );
    let Some(example) = examples
        .iter()
        .find(|example| example.supports_chip(chip) && example.matches_name(name))
    else {
        return Ok(None);
    };

    let package_path = windows_safe_path(&workspace.join(Package::Examples.directory()));
    let out_dir = workspace.join("target").join("binary-size");
    build_examples(
        Package::Examples,
        chip,
        false,
        toolchain,
        false,
        vec![example.clone()],
        &package_path,
        Some(&out_dir),
    )?;

    let elf = out_dir
        .join(chip.to_string())
        .join(example.output_file_name());
    binary_size::measure_elf(&elf).map(Some)
}

/// The version of the compiler that `build_examples` picks for `chip`.
fn rustc_version(workspace: &Path, chip: Chip, toolchain: Option<&str>) -> Result<String> {
    let toolchain = toolchain.or(chip.is_xtensa().then_some("esp"));

    let mut command = Command::new("rustc");
    if let Some(toolchain) = toolchain {
        command.arg(format!("+{toolchain}"));
    }
    let output = command
        .arg("--version")
        .current_dir(workspace)
        .output()
        .context("Failed to run rustc")?;
    ensure!(output.status.success(), "rustc --version failed");

    Ok(String::from_utf8_lossy(&output.stdout).trim().to_string())
}

fn git(workspace: &Path, args: &[&str]) -> Result<String> {
    let output = Command::new("git")
        .args(args)
        .current_dir(workspace)
        .output()
        .context("Failed to run git")?;
    ensure!(
        output.status.success(),
        "git {} failed: {}",
        args.join(" "),
        String::from_utf8_lossy(&output.stderr)
    );

    Ok(String::from_utf8_lossy(&output.stdout).trim().to_string())
}

fn read_json<T: serde::de::DeserializeOwned>(path: &Path) -> Result<T> {
    let contents = std::fs::read_to_string(path)
        .with_context(|| format!("Failed to read {}", path.display()))?;
    serde_json::from_str(&contents).with_context(|| format!("Failed to parse {}", path.display()))
}

fn write_json(path: &Path, value: &impl serde::Serialize) -> Result<()> {
    write_file(path, &serde_json::to_string_pretty(value)?)
}

fn write_file(path: &Path, contents: &str) -> Result<()> {
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent)
            .with_context(|| format!("Failed to create {}", parent.display()))?;
    }
    std::fs::write(path, contents).with_context(|| format!("Failed to write {}", path.display()))
}
