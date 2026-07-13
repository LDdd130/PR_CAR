/* @theme prcar */
@import 'default';

/*
 * PR_CAR deck theme — white + signal orange, flat.
 * Rules: solid surfaces only (no translucent cards), hairline borders,
 * shadows at most 0 1px 2px, gradients banned except nowhere (top bar is solid).
 * All deck-specific classes live here; slides.md stays content-only.
 */

:root {
  --prcar-bg: #ffffff;
  --prcar-bg-warm: #fdfaf6;
  --prcar-surface: #ffffff;
  --prcar-surface-muted: #faf7f2;
  --prcar-text: #1c1917;
  --prcar-muted: #57534e;
  --prcar-subtle: #78716c;
  --prcar-border: #e7e0d8;
  --prcar-border-strong: #d6cec3;
  --prcar-accent: #ea5a0c;
  --prcar-accent-bright: #ff6b1a;
  --prcar-accent-soft: #fff1e6;
  --prcar-accent-border: #f5cfae;
  --prcar-accent-ink: #9a3412;
  --prcar-critical-text: #b91c1c;
  --prcar-critical-bg: #fef2f2;
  --prcar-critical-border: #fecaca;
  --prcar-solved-text: #15803d;
  --prcar-solved-bg: #f0fdf4;
  --prcar-solved-border: #bbf7d0;
  --prcar-shadow: 0 1px 2px rgba(28, 25, 23, 0.06);
  --prcar-radius: 6px;
  --prcar-code-font: "SFMono-Regular", "Cascadia Code", "Roboto Mono", Consolas,
    "Liberation Mono", Menlo, monospace;
  font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Helvetica, Arial,
    sans-serif;
}

section {
  position: relative;
  width: 1280px;
  height: 720px;
  box-sizing: border-box;
  padding: 62px 72px 56px;
  background: var(--prcar-bg);
  color: var(--prcar-text);
  font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Helvetica, Arial,
    sans-serif;
  font-size: 30px;
  line-height: 1.48;
  letter-spacing: 0;
}

section::before {
  content: "";
  position: absolute;
  top: 0;
  left: 0;
  width: 100%;
  height: 6px;
  background: var(--prcar-accent);
}

section::after {
  right: 34px;
  bottom: 24px;
  color: var(--prcar-subtle);
  font-size: 16px;
  font-weight: 600;
}

h1,
h2,
h3,
h4 {
  margin: 0;
  color: var(--prcar-text);
  font-weight: 760;
  letter-spacing: 0;
}

h1 {
  max-width: 1080px;
  margin-bottom: 26px;
  font-size: 58px;
  line-height: 1.08;
}

h2 {
  position: relative;
  margin-bottom: 26px;
  padding-bottom: 14px;
  border-bottom: 1px solid var(--prcar-border);
  font-size: 44px;
  line-height: 1.14;
}

h2::after {
  content: "";
  position: absolute;
  left: 0;
  bottom: -1px;
  width: 88px;
  height: 4px;
  background: var(--prcar-accent);
}

h3 {
  margin-top: 22px;
  margin-bottom: 12px;
  font-size: 30px;
  line-height: 1.22;
}

h4 {
  margin-top: 16px;
  margin-bottom: 8px;
  color: var(--prcar-muted);
  font-size: 23px;
  line-height: 1.25;
  text-transform: uppercase;
}

p,
ul,
ol,
blockquote,
table,
pre {
  margin-top: 0;
}

p {
  margin-bottom: 18px;
  color: var(--prcar-text);
}

strong {
  color: var(--prcar-text);
  font-weight: 750;
}

em {
  color: var(--prcar-muted);
}

a {
  color: var(--prcar-accent);
  text-decoration: none;
}

a:hover {
  text-decoration: underline;
}

ul,
ol {
  margin-bottom: 18px;
  padding-left: 1.15em;
}

li {
  margin: 8px 0;
  padding-left: 0.1em;
}

li::marker {
  color: var(--prcar-accent);
  font-weight: 700;
}

blockquote {
  margin: 22px 0;
  padding: 18px 22px;
  border-left: 4px solid var(--prcar-accent);
  border-radius: 0 var(--prcar-radius) var(--prcar-radius) 0;
  background: var(--prcar-accent-soft);
  color: var(--prcar-text);
}

blockquote p:last-child,
li > p:last-child {
  margin-bottom: 0;
}

hr {
  height: 1px;
  margin: 28px 0;
  border: 0;
  background: var(--prcar-border);
}

table {
  display: table; /* default theme sets display:block — cells would not fill width */
  width: 100%;
  margin: 18px 0;
  border-collapse: collapse;
  overflow: hidden;
  border: 1px solid var(--prcar-border);
  border-radius: var(--prcar-radius);
  background: var(--prcar-surface);
  font-size: 22px;
}

th,
td {
  padding: 12px 16px;
  border-bottom: 1px solid var(--prcar-border);
  text-align: left;
  vertical-align: top;
}

th {
  border-bottom: 2px solid var(--prcar-border-strong);
  background: var(--prcar-surface-muted);
  color: var(--prcar-text);
  font-weight: 720;
}

tr:last-child td {
  border-bottom: 0;
}

img {
  max-width: 100%;
  border-radius: var(--prcar-radius);
}

code {
  border: 1px solid var(--prcar-accent-border);
  border-radius: 6px;
  background: var(--prcar-accent-soft);
  color: var(--prcar-accent-ink);
  font-family: var(--prcar-code-font);
  font-size: 0.82em;
}

pre {
  margin: 20px 0;
  padding: 20px 22px;
  border: 1px solid var(--prcar-border);
  border-radius: var(--prcar-radius);
  background: var(--prcar-surface-muted);
  box-shadow: var(--prcar-shadow);
}

pre code {
  display: block;
  padding: 0;
  border: 0;
  background: transparent;
  color: var(--prcar-text);
  font-family: var(--prcar-code-font);
  font-size: 21px;
  line-height: 1.55;
  tab-size: 2;
}

pre code,
pre code * {
  font-variant-ligatures: none;
}

/* GitHub-light highlight palette (unchanged) */
.hljs-comment,
.hljs-quote {
  color: #6e7781;
}

.hljs-keyword,
.hljs-selector-tag,
.hljs-subst {
  color: #cf222e;
}

.hljs-literal,
.hljs-number,
.hljs-tag .hljs-attr,
.hljs-template-variable,
.hljs-variable {
  color: #0550ae;
}

.hljs-doctag,
.hljs-string {
  color: #0a3069;
}

.hljs-title,
.hljs-section,
.hljs-selector-id {
  color: #8250df;
  font-weight: 650;
}

.hljs-type,
.hljs-class .hljs-title {
  color: #953800;
}

.hljs-tag,
.hljs-name,
.hljs-attribute {
  color: #116329;
}

.hljs-regexp,
.hljs-link {
  color: #116329;
}

.hljs-symbol,
.hljs-bullet {
  color: #953800;
}

.hljs-built_in,
.hljs-builtin-name {
  color: #0550ae;
}

.hljs-meta {
  color: #57606a;
}

.hljs-deletion {
  background: #ffebe9;
  color: #82071e;
}

.hljs-addition {
  background: #dafbe1;
  color: #116329;
}

mark {
  border-radius: 5px;
  background: #fff8c5;
  color: var(--prcar-text);
  box-decoration-break: clone;
  -webkit-box-decoration-break: clone;
}

.lead {
  color: var(--prcar-muted);
  font-size: 34px;
  line-height: 1.36;
}

.muted {
  color: var(--prcar-muted);
}

.small {
  font-size: 0.78em;
}

.compact {
  font-size: 24px;
  line-height: 1.4;
}

.caption {
  margin-top: 10px;
  color: var(--prcar-subtle);
  font-size: 18px;
}

.callout {
  padding: 18px 20px;
  border: 1px solid var(--prcar-accent-border);
  border-radius: var(--prcar-radius);
  background: var(--prcar-accent-soft);
}

.center {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  text-align: center;
}

.center h1,
.center h2,
.center p {
  margin-right: auto;
  margin-left: auto;
}

section.title {
  display: flex;
  flex-direction: column;
  justify-content: center;
  padding: 72px 86px;
  background: var(--prcar-bg-warm);
}

section.title h1 {
  max-width: 980px;
  margin-bottom: 22px;
  font-size: 66px;
}

section.title h1::after {
  content: "";
  display: block;
  width: 200px;
  height: 6px;
  margin-top: 24px;
  background: var(--prcar-accent);
}

section.title p {
  max-width: 900px;
  color: var(--prcar-muted);
  font-size: 32px;
}

section.inverse {
  background: var(--prcar-text);
  color: #fafaf9;
}

section.inverse::before {
  background: var(--prcar-accent-bright);
}

section.inverse h1,
section.inverse h2,
section.inverse h3,
section.inverse h4,
section.inverse p,
section.inverse strong {
  color: #ffffff;
}

section.inverse h2 {
  border-bottom-color: rgba(255, 255, 255, 0.16);
}

section.inverse .muted,
section.inverse em {
  color: #d6d3d1;
}

section.inverse code {
  border-color: rgba(255, 255, 255, 0.16);
  background: rgba(255, 255, 255, 0.1);
  color: #ffffff;
}

section.split {
  display: grid;
  grid-template-columns: repeat(2, minmax(0, 1fr));
  grid-auto-rows: max-content;
  gap: 24px 40px;
  align-content: start;
  align-items: start;
}

section.split > h1:first-child,
section.split > h2:first-child {
  grid-column: 1 / -1;
  margin-bottom: 4px;
}

section.split > h3,
section.split > h4,
section.split > p,
section.split > ul,
section.split > ol,
section.split > blockquote,
section.split > pre,
section.split > table,
section.split > div {
  min-width: 0;
}

section.split > pre {
  margin: 0;
}

section.split pre code {
  font-size: 18px;
  line-height: 1.48;
}

section.split table {
  font-size: 18px;
}

section.split .left {
  grid-column: 1;
}

section.split .right {
  grid-column: 2;
}

section.split .full {
  grid-column: 1 / -1;
}

section.split > .left,
section.split > .right:not(.placeholder) {
  padding: 20px 24px;
  border: 1px solid var(--prcar-border);
  border-radius: var(--prcar-radius);
  background: var(--prcar-surface);
  box-shadow: var(--prcar-shadow);
}

section.split > .left {
  border-left: 4px solid var(--prcar-accent);
}

section.split > .left > h3:first-child,
section.split > .right > h3:first-child {
  margin-top: 0;
}

section.code-focus {
  display: flex;
  flex-direction: column;
  gap: 18px;
  padding: 42px 54px 46px;
  background: var(--prcar-bg);
}

section.code-focus > h1:first-child,
section.code-focus > h2:first-child,
section.code-focus > h3:first-child {
  flex: 0 0 auto;
  margin-bottom: 0;
  padding-bottom: 12px;
  border-bottom: 1px solid var(--prcar-border);
  font-size: 36px;
  line-height: 1.16;
}

/* Keep pre as a plain block: marp-pre auto-scaling measures content width,
   and flex sizing here distorts that measurement into a huge downscale. */
section.code-focus > pre {
  flex: 0 0 auto;
  margin: 0;
  padding: 24px 28px;
  border: 1px solid var(--prcar-border);
  border-radius: var(--prcar-radius);
  background: var(--prcar-surface-muted);
  box-shadow: var(--prcar-shadow);
}

section.code-focus > pre code {
  font-size: 26px;
  line-height: 1.5;
  white-space: pre;
}

section.code-focus > p {
  flex: 0 0 auto;
  margin: 0;
  color: var(--prcar-muted);
  font-size: 22px;
}

/* Numbered card list: applies to a top-level ol on plain content slides
   (used by the agenda and principles slides). */
section:not(.split):not(.title):not(.code-focus):not(.inverse) > ol {
  display: grid;
  grid-template-columns: repeat(2, minmax(0, 1fr));
  gap: 14px 18px;
  padding-left: 0;
  list-style: none;
  counter-reset: deck-step;
}

section:not(.split):not(.title):not(.code-focus):not(.inverse) > ol > li {
  counter-increment: deck-step;
  min-height: 44px;
  margin: 0;
  padding: 13px 16px 13px 54px;
  border: 1px solid var(--prcar-border);
  border-radius: var(--prcar-radius);
  background: var(--prcar-bg-warm);
  box-shadow: var(--prcar-shadow);
  position: relative;
}

section:not(.split):not(.title):not(.code-focus):not(.inverse) > ol > li::before {
  content: counter(deck-step);
  position: absolute;
  left: 15px;
  top: 13px;
  display: grid;
  place-items: center;
  width: 28px;
  height: 28px;
  border-radius: 999px;
  background: var(--prcar-accent);
  color: #ffffff;
  font-size: 16px;
  font-weight: 800;
}

/* Visual placeholder block — working marker for images/diagrams to be added */
.placeholder {
  display: flex;
  position: relative;
  align-items: center;
  justify-content: center;
  min-height: 250px;
  padding: 22px;
  border: 2px dashed var(--prcar-accent-bright);
  border-radius: var(--prcar-radius);
  background: var(--prcar-accent-soft);
  color: var(--prcar-accent-ink);
  font-size: 22px;
  font-weight: 700;
  text-align: center;
}

.placeholder::before {
  content: "VISUAL";
  position: absolute;
  top: 14px;
  left: 14px;
  padding: 4px 11px;
  border: 1px solid var(--prcar-accent-border);
  border-radius: 999px;
  background: #ffffff;
  color: var(--prcar-accent);
  font-size: 13px;
  font-weight: 800;
  letter-spacing: 0.06em;
}

.placeholder.large {
  min-height: 390px;
}

.placeholder.flow {
  min-height: 330px;
}

.kpi-row {
  display: grid;
  grid-template-columns: repeat(3, minmax(0, 1fr));
  gap: 16px;
  margin-top: 28px;
}

.kpi {
  padding: 18px 20px;
  border: 1px solid var(--prcar-border);
  border-top: 4px solid var(--prcar-accent);
  border-radius: var(--prcar-radius);
  background: var(--prcar-surface);
  box-shadow: var(--prcar-shadow);
}

.kpi strong {
  display: block;
  margin-bottom: 2px;
  color: var(--prcar-accent);
  font-size: 36px;
  line-height: 1.1;
}

.kpi span {
  color: var(--prcar-muted);
  font-size: 18px;
  font-weight: 650;
}

.badge {
  display: inline-flex;
  align-items: center;
  min-height: 1.55em;
  margin: 0 0.12em;
  padding: 0.12em 0.48em;
  border: 1px solid var(--prcar-border);
  border-radius: 999px;
  background: var(--prcar-surface-muted);
  color: var(--prcar-muted);
  font-size: 0.58em;
  font-weight: 760;
  letter-spacing: 0.02em;
  line-height: 1;
  text-transform: uppercase;
  vertical-align: 0.12em;
  white-space: nowrap;
}

.badge.critical {
  border-color: var(--prcar-critical-border);
  background: var(--prcar-critical-bg);
  color: var(--prcar-critical-text);
}

.badge.solved {
  border-color: var(--prcar-solved-border);
  background: var(--prcar-solved-bg);
  color: var(--prcar-solved-text);
}
