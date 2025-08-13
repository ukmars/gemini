# Maintainer Guidance – Branching & Release Procedures

## Outline

We welcome contributions to the development of this project. The best way to do this if you are not a core maintainer is to fork the repository, create a branch and then submit a Pull Request.

Core maintainers will likely work directly with the repository and we have some guidelines for that.

## Purpose

This document explains how we work with our GitHub repository, including our branching model, pull request process, and release strategy. It is designed to help new maintainers quickly understand and follow our workflow.

---

## 1. Treat GitHub History as Permanent

Once something is pushed to GitHub, regard it as **permanent**.  
Rewriting shared history (for example, with `git push --force`) can cause serious problems for others, breaking their local copies or forks.  
Protected branches and pull requests ensure history stays consistent.

---

## 2. Default Workflow – Trunk‑Based Development

We use a **trunk‑based development** approach:

- **`main` branch**  
  Always correct, stable, and ready for production or test builds. No direct commits allowed — all changes must come via Pull Requests (PRs).

- **Short‑lived branches**  
  Used for a single, focused purpose, such as:
  - Minor feature
  - Bug fix
  - Issue resolution
  - Correction

- **Pull Requests**  
  - Every branch is merged into `main` only through a PR.  
  - Requires at least one other maintainer’s review.  
  - CI must pass before merging.  
  - PRs can be opened early and updated continuously during development.  
  - Without active review, PRs risk accumulating unreviewed changes — avoid letting them stagnate.

- **Commits**  
  Keep them frequent, focused, and meaningful to make progress tracking and conflict resolution easier.#

- **Submitting a Pull Request (as a maintainer)**  
  - Clone the repository to your local computer
  - Create a branch with a sensible name
  - Do your work, committing as appropriate
  - Push the branch to origin (on Github)
  - Create the Pull Request

---

## 3. Continuous Integration (CI)

Our CI checks ensure the project remains functional:

- A failing CI run blocks a merge into `main`.
- Checks include automated tests, build validation, and ECAD‑specific verifications.

---

## 4. Versioning

We use **Semantic Versioning** for tags:

- **Patch** (`V2.1.z`) – non‑breaking fixes  
- **Minor** (`V2.y.z`) – non‑breaking improvements  
- **Major** (`Vx.y.z`) – large, potentially incompatible changes

---

## 5. When to Use More Complex Branching

### Develop Branch
A `develop` branch can be maintained to integrate work before merging into `main`.  
However, this is harder to follow, maintain, and enforce, so it is rarely needed for our current team size and experience level.

### Special Revision Branches
For major planned revisions (for example, `V2.1.z` → `V2.2.z`):

- Create a temporary branch, e.g., `V2.2.dev`.
- Suitable for big changes such as adding a new sensor board or mezzanine board.
- Must be updated frequently from `main` to avoid painful conflicts later.
- Should be short‑lived and merged back as soon as the work is ready.
- These branches are the **exception**, not the norm.

---

## 6. Best Practices

- Protect `main` in GitHub settings:
  - Require PR reviews
  - Require passing CI
  - Prevent force pushes
- Keep branches alive for days, not weeks.
- One logical change per branch/PR.
- Coordinate on files that do not merge well (e.g., KiCad projects).
- Use Git LFS for large binary assets.

---

## Summary

Follow trunk‑based development as the default. Keep `main` clean, use short‑lived branches for changes, enforce review and CI checks, and tag releases using semantic versioning. Only introduce more complex workflows when they provide clear value for a specific large revision.
