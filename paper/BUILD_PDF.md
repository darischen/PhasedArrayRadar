# Building the PDF

This document provides instructions for building the research paper PDF from the LaTeX source.

## Requirements

You need:
- LaTeX distribution (TeX Live, MacTeX, or MiKTeX)
- latexmk (usually included with TeX distributions)
- BibTeX (usually included with LaTeX)

## Building on Your System

### Option 1: Using latexmk (Recommended)

```bash
cd paper
make pdf
```

This will:
1. Compile LaTeX source with pdflatex
2. Process bibliography with BibTeX
3. Re-compile to resolve cross-references
4. Generate `paper.pdf`

### Option 2: Manual Compilation

```bash
cd paper
pdflatex -interaction=nonstopmode paper.tex
bibtex paper
pdflatex -interaction=nonstopmode paper.tex
pdflatex -interaction=nonstopmode paper.tex
```

### Option 3: Using Docker

If you have Docker installed and running:

```bash
cd paper
docker run --rm -v "$(pwd):/latex" \
  texlive/texlive:latest \
  bash -c "cd /latex && latexmk -pdf -interaction=nonstopmode paper.tex"
```

## Cleaning Build Artifacts

```bash
cd paper
make clean      # Remove .aux, .log, .out, etc. (keep PDF)
make distclean   # Also remove paper.pdf
```

## Expected Output

- **Filename:** `paper.pdf`
- **Size:** ~500 KB - 2 MB
- **Page Count:** 11-12 pages
- **Figures:** 4 high-quality PDF plots embedded

## Troubleshooting

### "Missing package" errors

Example:
```
! LaTeX Error: File `natbib.sty' not found.
```

**Solution:** Install the missing package
- **TeX Live:** `tlmgr install natbib`
- **MiKTeX:** Use the MiKTeX Console
- **MacTeX:** Packages are usually pre-installed

### "pdflatex command not found"

**Solution:** Ensure your LaTeX distribution is in PATH
- **TeX Live:** Run `source /usr/local/texlive/YYYY/profile.d/texlive.sh`
- **MacTeX:** Should be automatic; verify with `which pdflatex`
- **MiKTeX:** Add `C:\Program Files\MiKTeX\miktex\bin` to PATH

### Bibliography not showing

Make sure all three compilation steps run:
1. `pdflatex` (creates .aux file)
2. `bibtex` (reads .bib and creates .bbl)
3. `pdflatex` (includes bibliography)

## LaTeX Source Files

- **`paper.tex`** — Main document with 7 sections
- **`preamble.tex`** — Package imports and styling
- **`paper.bib`** — Bibliography entries
- **`figures/`** — Directory for embedded figures (PDF format)

## Document Structure

The paper is organized into 7 sections:
1. **Introduction** — HB100 background and motivation
2. **Related Work** — Phased array literature and context
3. **System Design** — Hardware, firmware, and software architecture
4. **Signal Processing** — MUSIC, monopulse, EKF, forward-backward averaging
5. **Simulation & Validation** — Synthetic results and sensitivity analysis
6. **Reproducibility** — Software stack, figure generation, hardware reproduction
7. **Conclusion** — Summary, impact, and future work

## Version Information

- **Created:** 2025-05-30
- **LaTeX Type:** pdfLaTeX
- **Document Class:** article (10pt, 1.15 line spacing)
- **Bibliography Style:** abbrvnat (abbreviated natural)
