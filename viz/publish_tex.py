#!/usr/bin/env python3
"""
Pubblica i .tex generati nel repo del report, senza clonarlo.

Perche' esiste: i tre file di viz/out/tex/ vanno riportati nel repo del report a
ogni rigenerazione (bag nuova, taratura ritoccata). Clonare 42 MB per copiare
56 KB non ha senso, e trascinarli nella UI di GitHub e' un gesto manuale che si
sbaglia in silenzio — file dimenticato, cartella sbagliata, tre commit separati.

Usa la Git Data API tramite `gh`, quindi:
  - nessun clone, nessun token da gestire (l'autenticazione e' quella di gh);
  - UN SOLO commit con tutti e tre i file, non uno per file;
  - se un file non e' cambiato non viene ricommittato.

Uso:
    python3 viz/publish_tex.py --dry-run          # mostra cosa farebbe
    python3 viz/publish_tex.py                    # pubblica
    python3 viz/publish_tex.py --dir Latex_noc/metrics --branch main
"""
from __future__ import annotations

import argparse
import base64
import json
import os
import subprocess
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)

FILES = ("metrics_macros.tex", "metrics_body.tex", "metrics_standalone.tex")


def gh(path: str, method: str = "GET", body: dict | None = None):
    """Chiamata all'API di GitHub attraverso gh, che ci mette l'autenticazione."""
    cmd = ["gh", "api", "-X", method, path]
    if body is not None:
        cmd += ["--input", "-"]
    try:
        out = subprocess.run(
            cmd, input=json.dumps(body) if body else None,
            capture_output=True, text=True, check=True).stdout
    except FileNotFoundError:
        raise SystemExit("gh non installato: https://cli.github.com")
    except subprocess.CalledProcessError as exc:
        raise SystemExit(f"API {method} {path} fallita:\n{exc.stderr.strip()}")
    return json.loads(out) if out.strip() else {}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--repo", default="Relo02/NOC_report")
    ap.add_argument("--branch", default="main")
    ap.add_argument("--dir", default="Latex_noc/metrics",
                    help="cartella di destinazione nel repo del report")
    ap.add_argument("--src", default=os.path.join(_HERE, "out", "tex"))
    ap.add_argument("--message", default=None)
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    locali = {}
    for name in FILES:
        p = os.path.join(args.src, name)
        if not os.path.exists(p):
            raise SystemExit(f"manca {p}: eseguire prima  python3 viz/results_tex.py")
        locali[name] = open(p, "rb").read()

    # Provenienza: il commit deve dire da quale commit del repo del CODICE
    # nascono i numeri, altrimenti il .tex nel repo del report e' orfano.
    def git(*a):
        try:
            return subprocess.check_output(["git", *a], cwd=_ROOT,
                                           stderr=subprocess.DEVNULL).decode().strip()
        except Exception:
            return "?"
    sha_codice = git("rev-parse", "--short", "HEAD")
    branch_codice = git("rev-parse", "--abbrev-ref", "HEAD")
    sporco = bool(git("status", "--porcelain"))
    msg = args.message or (
        f"metrics: rigenerate da {branch_codice}@{sha_codice}"
        + (" (albero sporco)" if sporco else ""))

    ref = gh(f"/repos/{args.repo}/git/ref/heads/{args.branch}")
    base_commit = ref["object"]["sha"]
    base_tree = gh(f"/repos/{args.repo}/git/commits/{base_commit}")["tree"]["sha"]

    # Cosa e' gia' lassu': i file identici non vanno ricommittati.
    esistenti = {}
    try:
        for item in gh(f"/repos/{args.repo}/contents/{args.dir}?ref={args.branch}"):
            esistenti[item["name"]] = item["sha"]
    except SystemExit:
        pass                                    # la cartella non esiste ancora

    print(f"repo      {args.repo}  (branch {args.branch})")
    print(f"cartella  {args.dir}")
    print(f"messaggio {msg}\n")

    da_fare = []
    for name, data in locali.items():
        # lo sha di git di un blob e' sha1("blob <len>\0" + contenuto)
        import hashlib
        sha_locale = hashlib.sha1(b"blob %d\0" % len(data) + data).hexdigest()
        stato = ("invariato" if esistenti.get(name) == sha_locale
                 else ("aggiorna" if name in esistenti else "nuovo"))
        print(f"  {stato:<10} {name:<26} {len(data)/1024:6.1f} KB")
        if stato != "invariato":
            da_fare.append((name, data))

    if not da_fare:
        print("\nnulla da pubblicare: i file lassu' sono gia' identici.")
        return 0
    if args.dry_run:
        print(f"\n[dry-run] {len(da_fare)} file da pubblicare in un commit. "
              f"Rilanciare senza --dry-run.")
        return 0

    entries = []
    for name, data in da_fare:
        blob = gh(f"/repos/{args.repo}/git/blobs", "POST",
                  {"content": base64.b64encode(data).decode(), "encoding": "base64"})
        entries.append({"path": f"{args.dir}/{name}", "mode": "100644",
                        "type": "blob", "sha": blob["sha"]})

    tree = gh(f"/repos/{args.repo}/git/trees", "POST",
              {"base_tree": base_tree, "tree": entries})
    commit = gh(f"/repos/{args.repo}/git/commits", "POST",
                {"message": msg, "tree": tree["sha"], "parents": [base_commit]})
    gh(f"/repos/{args.repo}/git/refs/heads/{args.branch}", "PATCH",
       {"sha": commit["sha"]})

    print(f"\npubblicato: {commit['sha'][:10]}")
    print(f"https://github.com/{args.repo}/tree/{args.branch}/{args.dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
