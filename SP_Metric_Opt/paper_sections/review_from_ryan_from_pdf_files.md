Revision Plan
Optimizing Robot Computation for Safety and Performance in Dynamic Environments
Target venue: IEEE Transactions on Automation Science and Engineering (T-ASE)
June 2026
Purpose and scope
Revisions are split into two tiers:
• Category 1 — strictly necessary for the paper to be publishable at all.
• Category 2 — ideal but not strictly necessary for a venue like T-ASE.
Attribution tags: [All 3] = raised by all three reviews; [R1]/[R2] = external reviewer; [Me] =
internal review.
1 Category 1 — Strictly necessary
1.1 Presentation correctness
• Fix the Table I escaping bug: the cell renders the literal string \ith{i} instead of “i-th”
(main.tex:228). [Me]
• Proofread the confirmed typos: works works (main.tex:162), an robot (main.tex:128, 211),
aerocrafts (160), preferablely (571), parital (638), dentoes (415), pikes (1365), and temporarily→temporally (212, 356). [R1+R2+Me]
1.2 Core mathematical correctness
• Repair the safety definition and threshold convention. Define the deadline-miss probability as Pr(ri > Di), choose one meaning for Θi
, and make the Definition (main.tex:432), the
safety function, Example 2, and the experiment table mutually consistent. [All 3]
Efficiency note: verify the released code once; if it already computes the miss probability (as
Example 2’s intent indicates), this is a writing-only fix and all plots remain valid.
• Resolve the Θi contradiction in the experiment table. The definition (main.tex:436)
sets Θi to be the probability the system can tolerate a deadline miss — a maximum tolerable miss
probability, so small Θ means a stricter, more critical task. The table (main.tex:1022–1026)
and its text (main.tex:1007) do the reverse: the most safety-critical task, MPC, is given the
largest value, ΘMPC = 0.99, and “higher thresholds” are said to mark more important tasks.
Read through the definition, ΘMPC = 0.99 means “tolerate MPC missing 99% of the time”
— i.e. the least-protected task, the opposite of the stated intent. (The definition, the safety
function, and Example 2 all use this tolerable-miss reading; only the table and its surrounding
text use the reverse “required-reliability” reading, in which 0.99 would mean “must meet its
deadline 99% of the time.”) The table is sensible on its own — the clash is between it and the
paper’s own definition. Fix by adopting one convention everywhere:
1
A. Keep Θi = maximum tolerable miss probability: write safety as Pr(ri > Di) ≤ Θi and invert
the table so critical tasks get small values (e.g. MPC ≈ 0.01, SLAM ≈ 0.1). This changes
the input numbers, so it requires re-running.
B. Redefine Θi = minimum required probability of meeting the deadline: safety becomes Pr(ri ≤
Di) ≥ Θi (inequality flips), the table values stay as-is, and the column is renamed “required
success probability.” Text-only if the code already treats Θ this way.
Recommended: check the released code; if it already uses Θ as a reliability target (as the
experiment narrative implies), take B — a text-only fix with no re-run. Resolve this together
with the safety-definition item above, so the inequality direction, the symbol Pr(ri ≤ Di), and
Θi
’s meaning all agree. [Me]
• Fix the SP-metric equation (main.tex:478): the unmatched parenthesis, and the index
clash P
i wi →
P
j wj . [R1+Me]
• Correct Example 2’s arithmetic (main.tex:500–503): the two performance coefficients are
swapped, and the safety coefficient is written −0.1 vs. the defined −0.01. [Me]
• Fix the SP interpretability claim (main.tex:528): “threshold 0.9 ⇒ both safety and
performance ≥ 0.9” is true for a single product term PiSi ≥ 0.9 but false for the weighted
sum over tasks (a strong task masks a weak one). State it correctly or remove it. [R2]
• Define Normalize() precisely (domain, min/max, saturation) so the central metric is reproducible. [R2+Me]
• Fix the response-time-analysis definitions: hp(i) should be strictly higher priority, and the
response-time initialization must include the task’s own execution time Ci (main.tex:405, 413).
[R2+Me]
• Correct Algorithm 1 (modified Audsley) pseudocode (main.tex:634–657): shrink task pool
as tasks are assigned (otherwise duplicate assignments are possible); copy partial-assignment vectors before pushing (the subsequent Pop aliases and corrupts them); define the objective used by
SelectTop; relabel the output “Optimal”→“Selected”; and reconcile the “lowest-priority-first”
construction with the Definition that higher-priority tasks appear first. [R2]
1.3 Internal contradictions and results validity
• Fix the impossible RRT configuration: period = 10 ms vs. execution time stated as 1–3 s
elsewhere (main.tex:808 vs. 1025); a single job cannot meet its deadline. Sanity-check that all
periods exceed worst-case execution time. [Me+R2]
• Fix the simulation covariance matrix: with ρx,c, ρy,c each drawn from [−1, 1] independently,
the matrix is positive semidefinite only if ρ
2
x,c+ρ
2
y,c ≤ 1 (∼21% of sampled matrices are currently
invalid), and Gaussian sampling can yield negative execution times (main.tex:1259–1276).
Constrain the correlations and use a nonnegative distribution (truncated normal / lognormal).
[R2]
• Consolidate the headline result into a single number backed by a results table (currently
quoted as 15–50%, 20–50%, and 20–40% in different places). Uses data already collected.
[Me+R2]
1.4 Honest claim calibration
• Relabel the modified Audsley + beam search as a heuristic, not “optimal” (beam search
is not exhaustive). [All 3]
2
• Soften “guarantee” language and add a short assumptions/regimes statement: provable
bounds require a known/upper-bounded execution-time distribution; the unknown-environment
setting yields empirical/adaptive claims only. The paper already concedes this (main.tex:1224–
1225). [R2+Me]
• Soften “to the best of our knowledge, this is the first work. . . ” — it appears in the
same sentence as a citation to a close precedent (smARTflight, main.tex:182). Reframe as “an
adaptive SP-aware scheduling framework.” [All 3]
• Resolve the Gaussian-process overclaim the cheap way: remove GP from the abstract
and contribution list and describe the sliding-window predictor actually used (or explicitly mark
GP as an unused illustrative example). [All 3]
• Remove/correct the polar-coordinate footnote: “the math basically remains the same”
is false (a Gaussian does not stay Gaussian under a nonlinear polar transform). A one-line
deletion. [R1]
1.5 T-ASE format requirement
• Add the required “Note to Practitioners.” T-ASE mandates this plain-language section
alongside the abstract. Use it to lightly reframe the contribution toward automation scope and to
state the mapping between computational safety (deadline misses) and operational consequences.
[Venue + R1/R2]
2 Category 2 — Ideal, not strictly necessary for T-ASE
2.1 Strongly recommended despite being optional
The three items below are, strictly, Category 2, but are the most likely difference between “submittable” and “accepted” — and each is modest effort.
• ⋆ Add at least one credible adaptive / criticality-aware baseline (e.g. criticality-monotonic
priority, or the authors’ own prior RA-L’23 method). Beating only CFS and Rate-Monotonic is
the single most likely reason a reviewer still pushes back. [All 3]
• ⋆ Report task-level ground truth already logged: per-task deadline-miss rates, responsetime CDFs, SLAM error, path length, MPC tracking error, and scheduler overhead — this
directly answers the “evaluating only your own SP metric is circular” concern. Low effort if the
logs exist. [All 3]
• ⋆ Improve figure legibility. Figures 4–7 render at ≈2.29 in wide on the page (0.32×column
width), shrinking ≈10 pt labels to ≈3.6 pt; enlarge fonts and thin the box-plot density. [R1+R2]
2.2 Deeper evaluation
• Ablations (priority-only, configuration-only, prediction-only, no-incremental) plus an optimalitygap study vs. brute force, quantifying incremental “drift.” [R1+R2]
• Sensitivity sweeps over utilization, number of tasks, number of cores, environment-change rate,
and prediction error. [R2]
• A fuller baseline battery (EDF, deadline-monotonic, Audsley/OPA-for-schedulability, elastic
scheduling, static SP-optimal, periodic re-optimization, an oracle using future traces). [R1+R2]
• Confidence intervals / significance tests on the 50 runs. [R2+Me]
3
2.3 Robot-level / closed-loop validation
• A high-fidelity or physical scenario (e.g. Gazebo/CARLA/AirSim) demonstrating that scheduling decisions change robot-level outcomes (navigation success, tracking error, localization dropouts),
and a second robot workload. [R1+R2]
2.4 Added theory and rigor
• A formal probabilistic-schedulability result in the known-distribution regime, and a monotonicity
proposition for the SP metric. [R2]
• A self-contained RTA section with full assumptions plus an analytical-vs-measured miss-probability
validation on the Jetson traces. [R2]
• Actually implement and evaluate the GP predictor (vs. the Category-1 removal); concretely
instantiate and measure the environment vector Ek online. [R1+R2]
2.5 Positioning and polish
• A related-work comparison table adding Maxim et al. (RTNS 2011, Audsley for a probabilistic
objective), D´ıaz et al. (RTSS 2002), RED (RTSS 2023), Buttazzo et al. (elastic scheduling),
DMAC (ECRTS 2019), stochastic-DAG safety-performance (JSA 2024), and ROS 2 real-time
scheduling. [R1+R2+Me]
• Structural cleanup: merge the split results/analysis sections, relocate the Limitations section,
condense one-paragraph subsections. [All 3]
• A reproducibility checklist (hardware, kernel, CPU governor, commit, datasets). [R2]
4