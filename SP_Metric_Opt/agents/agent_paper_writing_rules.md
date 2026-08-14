# Paper Writing Rules

Guidelines for writing/editing LaTeX paper sections.

## 1. Do not re-explain ideas already covered in earlier sections
Each idea should be explained once, in the section where it is introduced.
When a later section touches the same idea, reference it (`\ref`/`\eqref`) or state it in one short clause --- do not re-derive or re-motivate it.
Wordiness from redundancy is the most common flaw: restating the in-search gating, the constraint, or the fallback logic that §6/§7 already established makes the writing read as padding.
If a sentence's content is already conveyed elsewhere, cut it.

## 2. Keep each line short
Write one sentence per source line (soft-wrap at ~90--100 chars max).
Long lines that fill the whole editor width are hard for a human to read and hard to diff.
Break a sentence across lines at natural clause boundaries if it would exceed the limit.
This makes edits and reviews separable line-by-line.

## 3. Save textbf for what matters the most, rather than as a simple bullet point