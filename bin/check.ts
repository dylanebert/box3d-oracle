#!/usr/bin/env bun
import { existsSync, readFileSync } from "node:fs";
import { join } from "node:path";

const required = ["README.md", "package.json", "bin/oracle.ts", "bin/check.ts", "test/oracle.test.ts"];
for (const path of required) {
  if (!existsSync(join(import.meta.dir, "..", path))) throw new Error(`missing required path: ${path}`);
}
const packageJson = JSON.parse(readFileSync(join(import.meta.dir, "..", "package.json"), "utf8")) as {
  scripts?: Record<string, string>;
};
for (const script of ["check", "test", "upstream-test"]) {
  if (!packageJson.scripts?.[script]) throw new Error(`package.json is missing ${script}`);
}
const oracle = readFileSync(join(import.meta.dir, "oracle.ts"), "utf8");
if (!oracle.includes("https://github.com/erincatto/box3d.git")) throw new Error("official source authority is not explicit");
if (oracle.includes("dylanebert/box3d")) throw new Error("fork source is present");
for (const forbidden of ["src/", "include/box3d", "expected output", "golden"]) {
  if (oracle.includes(forbidden)) throw new Error(`product or expected-output material is present: ${forbidden}`);
}
console.log("box3d-oracle check: PASS");
