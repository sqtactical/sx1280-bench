"""Turn one working-tree diff into a series of topic commits.

  python split_commits.py <plan.json>

plan.json:
  repo        git checkout at the base commit
  source      directory holding the final versions of the changed files ("" = the repo's working tree already has them)
  files       changed paths (relative, forward slashes); new files included
  git         path to git.exe
  commits     [{"message": "...", "rules": [{"path": "glob", "regex": "optional"}], "new_files": [...]}]
  profiler    {"message": "...", "new_files": [...], "whole_files": [...]}

Every change that belongs to the packet profiler (PROF_DO / PROF_SCOPE lines, #include "PacketProfiler.h",
#ifdef PROFILE_PACKETS blocks, and the files listed in profiler.whole_files) is held back for the last commit, so the
commits before it never reference the profiler. The other hunks go to the first commit whose rule matches; rules with a
regex are tried before path-only rules. A hunk no rule claims stops the run.
"""
import fnmatch, json, os, re, subprocess, sys

PROF_LINE = re.compile(r'PROF_DO\(|PROF_SCOPE\(|#include "(mesh/)?PacketProfiler\.h"')


def strip_profiler(text):
    out, skip = [], 0
    for line in text.splitlines(keepends=True):
        s = line.strip()
        if skip:
            if s.startswith("#if"):
                skip += 1
            elif s.startswith("#endif"):
                skip -= 1
            continue
        if s == "#ifdef PROFILE_PACKETS":
            skip = 1
            continue
        if PROF_LINE.search(line):
            continue
        out.append(line)
    return "".join(out)


def run(git, repo, *args, input_text=None):
    # bytes, not text mode: on Windows a text pipe would turn the patch's LF into CRLF
    data = input_text.encode("utf8") if input_text is not None else None
    r = subprocess.run([git, "-C", repo, *args], input=data, capture_output=True)
    out, err = r.stdout.decode("utf8", "replace"), r.stderr.decode("utf8", "replace")
    if r.returncode:
        raise SystemExit(f"git {' '.join(args)} failed:\n{out}\n{err}")
    return out


def read(path):
    with open(path, encoding="utf8", newline="") as f:
        return f.read()


def write(path, text):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf8", newline="") as f:
        f.write(text)


def split_hunks(diff):
    """[(path, header_lines, [hunk_text...])] from a unified diff"""
    files, cur = [], None
    for line in diff.splitlines(keepends=True):
        if line.startswith("diff --git "):
            cur = [line.split(" b/", 1)[1].strip(), [line], []]
            files.append(cur)
        elif cur and not cur[2] and not line.startswith("@@"):
            cur[1].append(line)
        elif line.startswith("@@"):
            cur[2].append(line)
        elif cur and cur[2]:
            cur[2][-1] += line
    return files


def main():
    plan = json.load(open(sys.argv[1], encoding="utf8"))
    repo, git, files = plan["repo"], plan["git"], plan["files"]
    prof = plan["profiler"]
    held_new = set(prof.get("new_files", []))
    whole = set(prof.get("whole_files", []))

    final = {f: read(os.path.join(plan["source"] or repo, f)) for f in files}
    tracked = set(run(git, repo, "ls-files").splitlines())
    if plan["source"]:
        for f in files:
            write(os.path.join(repo, f), final[f])

    # stage 1: everything except the profiler, split by topic
    for f in files:
        if f in held_new:
            os.remove(os.path.join(repo, f))
        elif f in whole and f in tracked:
            write(os.path.join(repo, f), run(git, repo, "show", "HEAD:" + f))
        else:
            write(os.path.join(repo, f), strip_profiler(final[f]))

    hunks = []
    for path, header, hs in split_hunks(run(git, repo, "diff", "-U3", "--no-color")):
        for h in hs:
            hunks.append((path, header, h))

    def changed(h):
        return "".join(l for l in h.splitlines(keepends=True)[1:] if l[:1] in "+-")

    owner = {}
    for pass_regex in (True, False):
        for i, (path, _, h) in enumerate(hunks):
            if i in owner:
                continue
            for ci, c in enumerate(plan["commits"]):
                if any(fnmatch.fnmatch(path, r["path"]) and (("regex" in r) == pass_regex) and
                       (not pass_regex or re.search(r["regex"], changed(h))) for r in c["rules"]):
                    owner[i] = ci
                    break
    missing = [hunks[i][0] + "\n" + hunks[i][2][:300] for i in range(len(hunks)) if i not in owner]
    if missing:
        raise SystemExit("unassigned hunks:\n\n" + "\n---\n".join(missing))

    for ci, c in enumerate(plan["commits"]):
        patch = ""
        for path in dict.fromkeys(p for i, (p, _, _) in enumerate(hunks) if owner[i] == ci):
            mine = [h for i, (p, _, h) in enumerate(hunks) if p == path and owner[i] == ci]
            header = next(hd for p, hd, _ in hunks if p == path)
            patch += "".join(header) + "".join(mine)
        if patch:
            run(git, repo, "apply", "--cached", "--recount", "-", input_text=patch)
        for nf in c.get("new_files", []):
            run(git, repo, "add", "--", nf)
        run(git, repo, "commit", "-q", "-F", "-", input_text=c["message"])
        print("committed:", c["message"].splitlines()[0])

    left = run(git, repo, "diff", "--stat") + run(git, repo, "status", "--porcelain", "--untracked-files=all", "--", *files)
    if left.strip():
        raise SystemExit("working tree differs from the topic commits:\n" + left)

    # stage 2: the profiler on top
    for f in files:
        write(os.path.join(repo, f), final[f])
    run(git, repo, "add", "--", *files)
    run(git, repo, "commit", "-q", "-F", "-", input_text=prof["message"])
    print("committed:", prof["message"].splitlines()[0])
    if run(git, repo, "status", "--porcelain", "--", *files).strip():
        raise SystemExit("final tree does not match the source")


if __name__ == "__main__":
    main()
