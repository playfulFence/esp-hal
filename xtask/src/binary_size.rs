//! Binary size of example firmware: measuring ELF files, and comparing two sets of measurements.

use std::{collections::BTreeMap, fmt::Write as _, path::Path};

use anyhow::{Context, Result, bail};
use object::{Object, ObjectSection, ObjectSegment, SectionFlags};
use serde::{Deserialize, Serialize};
use strum::IntoEnumIterator;

use crate::metadata::Chip;

/// A kind of memory the firmware occupies.
#[derive(
    Debug,
    Clone,
    Copy,
    PartialEq,
    Eq,
    PartialOrd,
    Ord,
    Hash,
    Serialize,
    Deserialize,
    strum::EnumIter,
)]
#[serde(rename_all = "kebab-case")]
pub enum Region {
    /// Everything stored in flash: code and read-only data used in place, and the initial
    /// contents of everything the bootloader loads into RAM.
    Flash,
    /// Statically allocated internal RAM.
    Ram,
    /// Statically allocated RTC memory.
    Rtc,
}

impl Region {
    fn label(self) -> &'static str {
        match self {
            Region::Flash => "Flash",
            Region::Ram => "RAM",
            Region::Rtc => "RTC",
        }
    }
}

/// Where the linker scripts place a section.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Placement {
    /// Reserves or pads address space. The size follows from the other sections or from the
    /// memory layout, not from what the firmware contains.
    Layout,
    /// Used in place from flash.
    Flash,
    Ram,
    Rtc,
}

fn placement(name: &str) -> Option<Placement> {
    const LAYOUT: &[&str] = &[
        ".stack",
        ".text_gap",
        ".rotext_dummy",
        ".rwdata_dummy",
        ".rtc_fast.dummy",
    ];
    const FLASH: &[&str] = &[".flash.appdesc", ".text", ".rodata"];
    const RAM: &[&str] = &[
        ".vectors",
        ".trap",
        ".rwtext",
        ".data",
        ".bss",
        ".noinit",
        ".dram2_uninit",
        ".dcache_reclaimed_uninit",
    ];

    // `.rodata` also covers `.rodata_merge` and `.rodata.wifi`.
    let is = |base: &str| {
        name.strip_prefix(base)
            .is_some_and(|rest| rest.is_empty() || rest.starts_with(['.', '_']))
    };

    if LAYOUT.contains(&name) {
        Some(Placement::Layout)
    } else if name.starts_with(".rtc_fast.") || name.starts_with(".rtc_slow.") {
        Some(Placement::Rtc)
    } else if FLASH.iter().any(|base| is(base)) {
        Some(Placement::Flash)
    } else if RAM.iter().any(|base| is(base)) {
        Some(Placement::Ram)
    } else {
        None
    }
}

/// The size of one firmware.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct Sizes {
    pub flash: u64,
    pub ram: u64,
    pub rtc: u64,
    /// The size of each allocated section.
    pub sections: BTreeMap<String, u64>,
}

impl Sizes {
    pub fn get(&self, region: Region) -> u64 {
        match region {
            Region::Flash => self.flash,
            Region::Ram => self.ram,
            Region::Rtc => self.rtc,
        }
    }
}

/// An allocated section of an ELF file.
#[derive(Debug, Clone)]
struct Section {
    name: String,
    address: u64,
    size: u64,
    /// Whether the section has contents, which the image stores in flash.
    has_contents: bool,
}

/// Measures the ELF file at `path`.
pub fn measure_elf(path: &Path) -> Result<Sizes> {
    let data = std::fs::read(path).with_context(|| format!("Failed to read {}", path.display()))?;
    let file = object::File::parse(&*data)
        .with_context(|| format!("Failed to parse {}", path.display()))?;

    let mut sections = Vec::new();
    for section in file.sections() {
        let SectionFlags::Elf { sh_flags } = section.flags() else {
            continue;
        };
        if sh_flags & u64::from(object::elf::SHF_ALLOC) == 0 || section.size() == 0 {
            continue;
        }
        sections.push(Section {
            name: section.name()?.to_string(),
            address: section.address(),
            size: section.size(),
            has_contents: section.file_range().is_some(),
        });
    }
    let segments: Vec<(u64, u64)> = file
        .segments()
        .map(|segment| (segment.address(), segment.address() + segment.size()))
        .collect();

    sizes_of(&sections, &segments).with_context(|| format!("Failed to measure {}", path.display()))
}

/// Adds up the sections. `segments` are the address ranges of the loadable segments.
fn sizes_of(sections: &[Section], segments: &[(u64, u64)]) -> Result<Sizes> {
    let segment_of = |section: &Section| {
        segments
            .iter()
            .position(|&(start, end)| (start..end).contains(&section.address))
    };

    let mut sizes = Sizes::default();
    for section in sections {
        // Input sections that no linker script rule matches end up as orphan sections next to
        // similar ones, in the same segment. Those take the placement of their neighbours.
        let placement = match placement(&section.name) {
            Some(placement) => placement,
            None => {
                let segment = segment_of(section);
                let mut neighbours = sections
                    .iter()
                    .filter(|other| segment.is_some() && segment_of(other) == segment)
                    .filter_map(|other| placement(&other.name))
                    .filter(|&p| p != Placement::Layout);
                match neighbours.next() {
                    Some(first) if neighbours.all(|other| other == first) => first,
                    _ => bail!(
                        "Can not tell where section `{}` is placed. Teach `binary_size::placement` about it.",
                        section.name
                    ),
                }
            }
        };

        match placement {
            Placement::Layout => continue,
            Placement::Flash => {}
            Placement::Ram => sizes.ram += section.size,
            Placement::Rtc => sizes.rtc += section.size,
        }
        if section.has_contents {
            sizes.flash += section.size;
        }
        *sizes.sections.entry(section.name.clone()).or_default() += section.size;
    }

    Ok(sizes)
}

/// The measurements of every configured example on one chip.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ChipSizes {
    pub chip: Chip,
    pub commit: String,
    /// `rustc --version` of the compiler that built the examples.
    pub rustc: String,
    pub examples: BTreeMap<String, Sizes>,
}

impl ChipSizes {
    /// Loads every `*.json` file in `dir`.
    pub fn load_dir(dir: &Path) -> Result<Vec<ChipSizes>> {
        let mut all = Vec::new();
        let entries =
            std::fs::read_dir(dir).with_context(|| format!("Failed to read {}", dir.display()))?;
        for entry in entries {
            let path = entry?.path();
            if path.extension().is_none_or(|ext| ext != "json") {
                continue;
            }
            let contents = std::fs::read_to_string(&path)
                .with_context(|| format!("Failed to read {}", path.display()))?;
            let sizes = serde_json::from_str(&contents)
                .with_context(|| format!("Failed to parse {}", path.display()))?;
            all.push(sizes);
        }
        all.sort_by_key(|sizes: &ChipSizes| sizes.chip);

        Ok(all)
    }
}

/// How much a region may grow before it counts as a regression.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Threshold {
    pub percent: f64,
    pub bytes: u64,
}

impl Threshold {
    /// Growth must pass both limits. The byte limit keeps small examples quiet, the percentage
    /// keeps large ones quiet.
    pub fn exceeded(&self, base: u64, head: u64) -> bool {
        let growth = head.saturating_sub(base);
        growth > 0 && growth >= self.bytes && growth as f64 * 100.0 >= self.percent * base as f64
    }
}

/// Which examples to measure, and when to report their growth.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Config {
    pub examples: Vec<String>,
    /// Regions without a threshold are reported, but never count as regressed.
    pub thresholds: BTreeMap<Region, Threshold>,
}

impl Config {
    pub fn load(path: &Path) -> Result<Self> {
        let contents = std::fs::read_to_string(path)
            .with_context(|| format!("Failed to read {}", path.display()))?;
        serde_json::from_str(&contents)
            .with_context(|| format!("Failed to parse {}", path.display()))
    }
}

/// A region of one example that grew past its threshold.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Regression {
    pub chip: Chip,
    pub example: String,
    pub region: Region,
    pub base: u64,
    pub head: u64,
}

impl Regression {
    fn growth_ratio(&self) -> f64 {
        (self.head - self.base) as f64 / self.base.max(1) as f64
    }
}

/// The result of comparing the measurements of two commits.
#[derive(Debug)]
pub struct Comparison {
    /// Ordered from the largest relative growth to the smallest.
    pub regressions: Vec<Regression>,
    pub report: String,
}

pub fn compare(base: &[ChipSizes], head: &[ChipSizes], config: &Config) -> Comparison {
    let base_by_chip: BTreeMap<Chip, &ChipSizes> = base.iter().map(|s| (s.chip, s)).collect();

    let mut regressions = Vec::new();
    let mut table = String::new();
    let mut toolchains = String::new();

    let regions: Vec<Region> = Region::iter().collect();
    writeln!(
        table,
        "| Chip | Example | {} |",
        regions
            .iter()
            .map(|r| format!("{} (bytes)", r.label()))
            .collect::<Vec<_>>()
            .join(" | ")
    )
    .unwrap();
    writeln!(table, "|---|---|{}", "---|".repeat(regions.len())).unwrap();

    for chip_head in head {
        let chip_base = base_by_chip.get(&chip_head.chip);
        if let Some(chip_base) = chip_base
            && chip_base.rustc != chip_head.rustc
        {
            writeln!(
                toolchains,
                "- `{}`: `{}` → `{}`",
                chip_head.chip, chip_base.rustc, chip_head.rustc
            )
            .unwrap();
        }

        for (example, sizes) in &chip_head.examples {
            let example_base = chip_base.and_then(|b| b.examples.get(example));
            let cells = regions.iter().map(|&region| {
                let now = sizes.get(region);
                let Some(was) = example_base.map(|b| b.get(region)) else {
                    return format!("{now} (new)");
                };
                let regressed = config
                    .thresholds
                    .get(&region)
                    .is_some_and(|t| t.exceeded(was, now));
                if regressed {
                    regressions.push(Regression {
                        chip: chip_head.chip,
                        example: example.clone(),
                        region,
                        base: was,
                        head: now,
                    });
                }
                let cell = format_change(was, now);
                if regressed {
                    format!("**{cell}**")
                } else {
                    cell
                }
            });
            let cells = cells.collect::<Vec<_>>().join(" | ");
            writeln!(table, "| `{}` | `{example}` | {cells} |", chip_head.chip).unwrap();
        }
    }

    regressions.sort_by(|a, b| b.growth_ratio().total_cmp(&a.growth_ratio()));

    let mut report = String::from("## Binary size report\n\n");
    match (base.first(), head.first()) {
        (Some(base), Some(head)) => writeln!(
            report,
            "Base: {}, head: {}.\n",
            short_sha(&base.commit),
            short_sha(&head.commit)
        )
        .unwrap(),
        (None, Some(head)) => writeln!(
            report,
            "Head: {}. There is nothing to compare against yet.\n",
            short_sha(&head.commit)
        )
        .unwrap(),
        _ => {}
    }
    if regressions.is_empty() {
        report.push_str("No region grew past its threshold.\n\n");
    } else {
        writeln!(
            report,
            "{} region(s) grew past their threshold, marked in bold.\n",
            regressions.len()
        )
        .unwrap();
    }
    report.push_str(&table);
    if !toolchains.is_empty() {
        report.push_str("\n### Toolchain changes\n\n");
        report.push_str(&toolchains);
    }

    Comparison {
        regressions,
        report,
    }
}

fn format_change(base: u64, head: u64) -> String {
    if base == head {
        return head.to_string();
    }
    let delta = head as i64 - base as i64;
    let percent = delta as f64 * 100.0 / base.max(1) as f64;
    format!("{head} ({delta:+}, {percent:+.2}%)")
}

fn short_sha(sha: &str) -> &str {
    &sha[..sha.len().min(10)]
}

/// A commit on the first-parent history of a branch.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Commit {
    pub sha: String,
    pub subject: String,
    /// The pull request a squash merge came from, read from the `(#123)` suffix of the subject.
    pub pr: Option<u64>,
}

impl Commit {
    fn parse(line: &str) -> Option<Self> {
        let (sha, subject) = line.split_once('\x1f')?;
        let pr = subject
            .strip_suffix(')')
            .and_then(|s| s.rsplit_once("(#"))
            .and_then(|(_, number)| number.parse().ok());

        Some(Self {
            sha: sha.to_string(),
            subject: subject.to_string(),
            pr,
        })
    }

    /// One markdown list item that names the commit.
    fn list_item(&self) -> String {
        // Commit subjects may @-mention people. The issue must not notify them.
        let subject = self.subject.replace('@', "@\u{200B}");
        match self.pr {
            Some(pr) => {
                let suffix = format!("(#{pr})");
                let subject = subject.strip_suffix(&suffix).unwrap_or(&subject).trim_end();
                format!("- #{pr}: {subject}")
            }
            None => format!("- {}: {subject}", short_sha(&self.sha)),
        }
    }
}

/// The commits in `base..head` on the first-parent history, oldest first.
pub fn first_parent_commits(workspace: &Path, base: &str, head: &str) -> Result<Vec<Commit>> {
    let output = std::process::Command::new("git")
        .args(["log", "--first-parent", "--reverse", "--format=%H%x1f%s"])
        .arg(format!("{base}..{head}"))
        .current_dir(workspace)
        .output()
        .context("Failed to run git log")?;
    if !output.status.success() {
        bail!(
            "git log {base}..{head} failed: {}",
            String::from_utf8_lossy(&output.stderr)
        );
    }

    Ok(String::from_utf8_lossy(&output.stdout)
        .lines()
        .filter_map(Commit::parse)
        .collect())
}

/// Finds the index of the first of `len` commits for which `regressed` holds.
///
/// Expects `regressed` to hold for the last commit, and to keep holding once it holds. Only calls
/// `regressed` for commits before the last one.
pub fn first_regressed(
    len: usize,
    mut regressed: impl FnMut(usize) -> Result<bool>,
) -> Result<usize> {
    if len == 0 {
        bail!("There are no commits to search");
    }

    let (mut low, mut high) = (0, len - 1);
    while low < high {
        let middle = low + (high - low) / 2;
        if regressed(middle)? {
            high = middle;
        } else {
            low = middle + 1;
        }
    }

    Ok(low)
}

/// The commit that made a regression pass its threshold.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Culprit {
    pub regression: Regression,
    pub commit: Commit,
}

/// The body of an issue that reports regressions.
pub fn issue_body(
    regressions: &[Regression],
    commits: &[Commit],
    culprit: Option<&Culprit>,
    report: &str,
    run_url: Option<&str>,
) -> String {
    let mut body = String::from(
        "The nightly binary size check found examples that grew past their threshold on `main`.\n\n",
    );
    if let Some(url) = run_url {
        writeln!(body, "Workflow run: {url}\n").unwrap();
    }

    body.push_str("### Likely cause\n\n");
    match culprit {
        Some(culprit) => writeln!(
            body,
            "Bisecting {} usage of `{}` on `{}` points at:\n\n{}\n",
            culprit.regression.region.label(),
            culprit.regression.example,
            culprit.regression.chip,
            culprit.commit.list_item()
        )
        .unwrap(),
        None => body.push_str(
            "Bisecting did not find a single commit. One of the commits below caused the growth.\n\n",
        ),
    }

    body.push_str("### Regressions\n\n");
    body.push_str("| Chip | Example | Region | Base (bytes) | Head (bytes) | Change |\n");
    body.push_str("|---|---|---|---|---|---|\n");
    for r in regressions {
        writeln!(
            body,
            "| `{}` | `{}` | {} | {} | {} | {:+} ({:+.2}%) |",
            r.chip,
            r.example,
            r.region.label(),
            r.base,
            r.head,
            r.head as i64 - r.base as i64,
            r.growth_ratio() * 100.0
        )
        .unwrap();
    }

    body.push_str("\n### Commits in this range\n\n");
    for commit in commits {
        body.push_str(&commit.list_item());
        body.push('\n');
    }

    body.push_str("\n<details>\n<summary>Full report</summary>\n\n");
    body.push_str(report.trim_end());
    body.push_str("\n\n</details>\n");

    body
}

#[cfg(test)]
mod tests {
    use pretty_assertions::assert_eq;

    use super::*;

    #[test]
    fn placement_of_linker_script_sections() {
        let cases = [
            (".text", Some(Placement::Flash)),
            (".rodata", Some(Placement::Flash)),
            (".rodata_merge", Some(Placement::Flash)),
            (".rodata.wifi", Some(Placement::Flash)),
            (".flash.appdesc", Some(Placement::Flash)),
            (".text_gap", Some(Placement::Layout)),
            (".rotext_dummy", Some(Placement::Layout)),
            (".rwdata_dummy", Some(Placement::Layout)),
            (".stack", Some(Placement::Layout)),
            (".rtc_fast.dummy", Some(Placement::Layout)),
            (".trap", Some(Placement::Ram)),
            (".vectors", Some(Placement::Ram)),
            (".rwtext.wifi", Some(Placement::Ram)),
            (".data.wifi", Some(Placement::Ram)),
            (".bss", Some(Placement::Ram)),
            (".dram2_uninit.bss", Some(Placement::Ram)),
            (".rtc_fast.text", Some(Placement::Rtc)),
            (".rtc_slow.bss", Some(Placement::Rtc)),
            (".textual", None),
            (".something_else", None),
        ];
        for (name, expected) in cases {
            assert_eq!(placement(name), expected, "{name}");
        }
    }

    fn section(name: &str, address: u64, size: u64, has_contents: bool) -> Section {
        Section {
            name: name.to_string(),
            address,
            size,
            has_contents,
        }
    }

    #[test]
    fn sizes_add_up_by_placement() {
        let sections = [
            section(".trap", 0x4080_0000, 0x100, true),
            section(".data", 0x4080_0100, 0x20, true),
            section(".bss", 0x4080_0120, 0x40, false),
            section(".rodata", 0x4200_0020, 0x300, true),
            section(".text_gap", 0x4200_0320, 0x1000, false),
            section(".text", 0x4201_0020, 0x2000, true),
            // An orphan in the same segment as `.text`.
            section(".iram1.5", 0x4201_2020, 0x80, true),
            section(".rtc_fast.text", 0x5000_0000, 0x8, true),
            section(".stack", 0x4080_0160, 0x4_0000, false),
        ];
        let segments = [
            (0x4080_0000, 0x4080_0160),
            (0x4200_0020, 0x4201_0020),
            (0x4201_0020, 0x4201_20a0),
            (0x5000_0000, 0x5000_0008),
            (0x4080_0160, 0x4084_0160),
        ];

        let sizes = sizes_of(&sections, &segments).unwrap();

        assert_eq!(sizes.flash, 0x100 + 0x20 + 0x300 + 0x2000 + 0x80 + 0x8);
        assert_eq!(sizes.ram, 0x100 + 0x20 + 0x40);
        assert_eq!(sizes.rtc, 0x8);
        assert_eq!(sizes.sections[".iram1.5"], 0x80);
        assert!(!sizes.sections.contains_key(".stack"));
    }

    #[test]
    fn unknown_section_without_known_neighbours_fails() {
        let sections = [
            section(".text", 0x4201_0020, 0x2000, true),
            section(".mystery", 0x4300_0000, 0x10, true),
        ];
        let segments = [(0x4201_0020, 0x4201_2020), (0x4300_0000, 0x4300_0010)];

        assert!(sizes_of(&sections, &segments).is_err());
    }

    #[test]
    fn threshold_needs_both_limits() {
        let threshold = Threshold {
            percent: 1.0,
            bytes: 100,
        };
        // 100 bytes, but only 0.1%.
        assert!(!threshold.exceeded(100_000, 100_100));
        // 10%, but only 50 bytes.
        assert!(!threshold.exceeded(500, 550));
        assert!(threshold.exceeded(10_000, 10_100));
        assert!(!threshold.exceeded(10_100, 10_000));
    }

    #[test]
    fn config_parses() {
        let config: Config = serde_json::from_str(
            r#"{
                "examples": ["hello_world"],
                "thresholds": { "flash": { "percent": 1.0, "bytes": 2048 } }
            }"#,
        )
        .unwrap();
        assert_eq!(config.examples, ["hello_world"]);
        assert_eq!(config.thresholds[&Region::Flash].bytes, 2048);
        assert!(!config.thresholds.contains_key(&Region::Ram));
    }

    fn chip_sizes(
        chip: Chip,
        commit: &str,
        rustc: &str,
        examples: &[(&str, u64, u64)],
    ) -> ChipSizes {
        ChipSizes {
            chip,
            commit: commit.to_string(),
            rustc: rustc.to_string(),
            examples: examples
                .iter()
                .map(|&(name, flash, ram)| {
                    (
                        name.to_string(),
                        Sizes {
                            flash,
                            ram,
                            ..Default::default()
                        },
                    )
                })
                .collect(),
        }
    }

    fn config() -> Config {
        Config {
            examples: vec![],
            thresholds: [
                (
                    Region::Flash,
                    Threshold {
                        percent: 1.0,
                        bytes: 1000,
                    },
                ),
                (
                    Region::Ram,
                    Threshold {
                        percent: 1.0,
                        bytes: 100,
                    },
                ),
            ]
            .into(),
        }
    }

    #[test]
    fn compare_finds_regressions() {
        let base = [chip_sizes(
            Chip::Esp32c6,
            "aaaaaaaaaaaa",
            "rustc 1.0",
            &[
                ("hello_world", 50_000, 4000),
                ("embassy_dhcp", 500_000, 60_000),
            ],
        )];
        let head = [chip_sizes(
            Chip::Esp32c6,
            "bbbbbbbbbbbb",
            "rustc 1.1",
            &[
                ("hello_world", 52_000, 4000),
                ("embassy_dhcp", 501_000, 61_000),
                ("bas_peripheral", 300_000, 50_000),
            ],
        )];

        let comparison = compare(&base, &head, &config());

        assert_eq!(
            comparison.regressions,
            [
                Regression {
                    chip: Chip::Esp32c6,
                    example: "hello_world".to_string(),
                    region: Region::Flash,
                    base: 50_000,
                    head: 52_000,
                },
                Regression {
                    chip: Chip::Esp32c6,
                    example: "embassy_dhcp".to_string(),
                    region: Region::Ram,
                    base: 60_000,
                    head: 61_000,
                },
            ]
        );
        assert_eq!(
            comparison.report,
            "## Binary size report\n\n\
             Base: aaaaaaaaaa, head: bbbbbbbbbb.\n\n\
             2 region(s) grew past their threshold, marked in bold.\n\n\
             | Chip | Example | Flash (bytes) | RAM (bytes) | RTC (bytes) |\n\
             |---|---|---|---|---|\n\
             | `esp32c6` | `bas_peripheral` | 300000 (new) | 50000 (new) | 0 (new) |\n\
             | `esp32c6` | `embassy_dhcp` | 501000 (+1000, +0.20%) | **61000 (+1000, +1.67%)** | 0 |\n\
             | `esp32c6` | `hello_world` | **52000 (+2000, +4.00%)** | 4000 | 0 |\n\
             \n### Toolchain changes\n\n\
             - `esp32c6`: `rustc 1.0` → `rustc 1.1`\n"
        );
    }

    #[test]
    fn compare_without_base() {
        let head = [chip_sizes(
            Chip::Esp32,
            "bbbbbbbbbbbb",
            "rustc 1.1",
            &[("hello_world", 52_000, 4000)],
        )];

        let comparison = compare(&[], &head, &config());

        assert!(comparison.regressions.is_empty());
        assert!(
            comparison
                .report
                .contains("There is nothing to compare against yet.")
        );
    }

    #[test]
    fn commit_reads_pull_request_number() {
        let commit = Commit::parse("abc\x1fTWAI: fix wakeups (#6421)").unwrap();
        assert_eq!(commit.pr, Some(6421));
        assert_eq!(commit.list_item(), "- #6421: TWAI: fix wakeups");

        let commit = Commit::parse("0123456789abcdef\x1fThanks @someone (no PR)").unwrap();
        assert_eq!(commit.pr, None);
        assert_eq!(
            commit.list_item(),
            "- 0123456789: Thanks @\u{200B}someone (no PR)"
        );
    }

    #[test]
    fn first_regressed_searches_in_halves() {
        for len in 1..20 {
            for culprit in 0..len {
                let mut probes = 0;
                let found = first_regressed(len, |i| {
                    assert!(i < len - 1, "probed the last commit");
                    probes += 1;
                    Ok(i >= culprit)
                })
                .unwrap();
                assert_eq!(found, culprit);
                assert!(probes <= len.ilog2() as usize + 1);
            }
        }
        assert!(first_regressed(0, |_| Ok(true)).is_err());
    }
}
