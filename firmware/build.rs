use std::process::Command;

fn main() {
    embuild::espidf::sysenv::output();

    // 펌웨어 버전 문자열 (FW_VERSION).
    // - 릴리스 태그(v<Cargo 버전>)에서 깨끗하게 빌드: "0.1.0"
    // - 그 밖: "0.1.0+<커밋>", 커밋하지 않은 변경이 있으면 "0.1.0+<커밋>-dirty"
    // git이 없거나 저장소 밖(소스 압축본)에서 빌드하면 "0.1.0+unknown".
    let version = std::env::var("CARGO_PKG_VERSION").unwrap();
    let describe = git(&["describe", "--tags", "--always", "--dirty", "--match", "v*"]);
    let hash = git(&["rev-parse", "--short", "HEAD"]);
    let dirty = describe.as_deref().is_some_and(|d| d.ends_with("-dirty"));
    let fw_version = match (describe.as_deref(), hash.as_deref()) {
        (Some(d), _) if d == format!("v{version}") => version,
        (_, Some(h)) => format!("{version}+{h}{}", if dirty { "-dirty" } else { "" }),
        _ => format!("{version}+unknown"),
    };
    println!("cargo:rustc-env=FW_VERSION={fw_version}");

    // 커밋·태그·스테이징이 바뀌거나 소스를 고치면 다시 계산한다
    if let Some(git_dir) = git(&["rev-parse", "--git-dir"]) {
        for path in ["HEAD", "refs", "index", "packed-refs"] {
            println!("cargo:rerun-if-changed={git_dir}/{path}");
        }
    }
    println!("cargo:rerun-if-changed=src");
}

fn git(args: &[&str]) -> Option<String> {
    let out = Command::new("git").args(args).output().ok()?;
    out.status
        .success()
        .then(|| String::from_utf8_lossy(&out.stdout).trim().to_string())
}
