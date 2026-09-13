#!/usr/bin/env bun
import { existsSync, readFileSync } from "node:fs";
import { join } from "node:path";

const required = ["README.md", "package.json", "bin/oracle.ts", "bin/check.ts", "test/oracle.test.ts", "include/box3d_oracle_adapter.h", "adapter/main.c", "adapter/math.c", "adapter/geometry.c", "adapter/distance.c", "adapter/tree.c", "adapter/manifold.c", "adapter/query.c", "adapter/mover.c", "adapter/writer.c", "schema/v1.json"];
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
if (oracle.includes("shallot/src") || oracle.includes("gold.json") || oracle.includes("expected output")) throw new Error("generation imports Shallot or expected-output material");
for (const source of ["adapter/math.c", "adapter/geometry.c", "adapter/distance.c", "adapter/tree.c", "adapter/manifold.c", "adapter/query.c", "adapter/mover.c"]) {
  const text = readFileSync(join(import.meta.dir, "..", source), "utf8");
  if (text.includes("#include \"src/") || text.includes("#include <src/")) throw new Error(`private upstream include in ${source}`);
}
const schema = JSON.parse(readFileSync(join(import.meta.dir, "..", "schema/v1.json"), "utf8")) as { $id?: string; description?: string };
if (schema.$id !== "box3d-oracle/v1" || !schema.description?.includes("fixed-width")) throw new Error("integer-preserving schema is incomplete");
console.log("box3d-oracle check: PASS");
