/* @theme prcar */
@import 'default';

:root {
  --prcar-bg: #f6f8fa;
  --prcar-surface: #ffffff;
  --prcar-surface-muted: #f8fafc;
  --prcar-text: #24292f;
  --prcar-muted: #57606a;
  --prcar-subtle: #6e7781;
  --prcar-border: #d0d7de;
  --prcar-border-soft: #eaeef2;
  --prcar-accent: #0969da;
  --prcar-accent-soft: #ddf4ff;
  --prcar-critical-text: #a40e26;
  --prcar-critical-bg: #fff1f2;
  --prcar-critical-border: #ffccd5;
  --prcar-solved-text: #116329;
  --prcar-solved-bg: #dafbe1;
  --prcar-solved-border: #aceebb;
  --prcar-shadow: 0 18px 48px rgba(31, 35, 40, 0.08);
  --prcar-radius: 8px;
  --prcar-code-font: "SFMono-Regular", "Cascadia Code", "Roboto Mono", Consolas,
    "Liberation Mono", Menlo, monospace;
  font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Helvetica, Arial,
    sans-serif;
}

section {
  width: 1280px;
  height: 720px;
  box-sizing: border-box;
  padding: 60px 72px;
  background:
    linear-gradient(180deg, rgba(255, 255, 255, 0.96), rgba(246, 248, 250, 0.98)),
    var(--prcar-bg);
  color: var(--prcar-text);
  font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Helvetica, Arial,
    sans-serif;
  font-size: 30px;
  line-height: 1.48;
  letter-spacing: 0;
}

section::after {
  right: 34px;
  bottom: 26px;
  color: #8c959f;
  font-size: 16px;
  font-weight: 600;
}

h1,
h2,
h3,
h4 {
  margin: 0;
  color: #1f2328;
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
  margin-bottom: 24px;
  padding-bottom: 14px;
  border-bottom: 1px solid var(--prcar-border-soft);
  font-size: 44px;
  line-height: 1.14;
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
  color: #1f2328;
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
  border-left: 5px solid var(--prcar-accent);
  border-radius: 0 var(--prcar-radius) var(--prcar-radius) 0;
  background: var(--prcar-accent-soft);
  color: #1f2328;
}

blockquote p:last-child,
li > p:last-child {
  margin-bottom: 0;
}

hr {
  height: 1px;
  margin: 28px 0;
  border: 0;
  background: var(--prcar-border-soft);
}

table {
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
  border-bottom: 1px solid var(--prcar-border-soft);
  text-align: left;
  vertical-align: top;
}

th {
  background: var(--prcar-surface-muted);
  color: #1f2328;
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
  border: 1px solid rgba(208, 215, 222, 0.75);
  border-radius: 6px;
  background: rgba(234, 238, 242, 0.72);
  color: #24292f;
  font-family: var(--prcar-code-font);
  font-size: 0.82em;
}

pre {
  margin: 20px 0;
  padding: 20px 22px;
  border: 1px solid var(--prcar-border);
  border-radius: var(--prcar-radius);
  background: #f6f8fa;
  box-shadow: 0 1px 0 rgba(31, 35, 40, 0.03);
}

pre code {
  display: block;
  padding: 0;
  border: 0;
  background: transparent;
  color: #24292f;
  font-family: var(--prcar-code-font);
  font-size: 21px;
  line-height: 1.55;
  tab-size: 2;
}

pre code,
pre code * {
  font-variant-ligatures: none;
}

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
  color: #24292f;
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
  background: linear-gradient(180deg, #ffffff 0%, #f6f8fa 100%);
}

section.title h1 {
  max-width: 980px;
  margin-bottom: 22px;
  font-size: 66px;
}

section.title p {
  max-width: 900px;
  color: var(--prcar-muted);
  font-size: 32px;
}

section.inverse {
  background: #1f2328;
  color: #f6f8fa;
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
  color: #c9d1d9;
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
  gap: 24px 48px;
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

section.code-focus {
  display: flex;
  flex-direction: column;
  gap: 18px;
  padding: 42px 54px 46px;
  background: #ffffff;
}

section.code-focus > h1:first-child,
section.code-focus > h2:first-child,
section.code-focus > h3:first-child {
  flex: 0 0 auto;
  margin-bottom: 0;
  padding-bottom: 12px;
  border-bottom: 1px solid var(--prcar-border-soft);
  font-size: 36px;
  line-height: 1.16;
}

section.code-focus > pre {
  display: flex;
  flex: 1 1 auto;
  min-height: 0;
  margin: 0;
  padding: 24px 28px;
  border: 1px solid var(--prcar-border);
  border-radius: var(--prcar-radius);
  background: #f6f8fa;
  box-shadow: var(--prcar-shadow);
}

section.code-focus > pre code {
  flex: 1 1 auto;
  overflow: hidden;
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

.badge {
  display: inline-flex;
  align-items: center;
  min-height: 1.55em;
  margin: 0 0.12em;
  padding: 0.12em 0.48em;
  border: 1px solid var(--prcar-border);
  border-radius: 999px;
  background: #f6f8fa;
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

