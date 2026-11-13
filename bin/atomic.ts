import { mkdirSync, mkdtempSync, renameSync, rmSync, writeFileSync } from "node:fs";
import { basename, dirname, join } from "node:path";

export function atomicWrite(path: string, data: string | Uint8Array): void {
  mkdirSync(dirname(path), { recursive: true });
  const work = mkdtempSync(join(dirname(path), `.${basename(path)}-`));
  try {
    const staged = join(work, "value");
    writeFileSync(staged, data);
    renameSync(staged, path);
  } finally { rmSync(work, { recursive: true, force: true }); }
}

/** No shared checkout, objects or output file. The first complete directory wins publication. */
export function publishBuild(path: string, build: (work: string) => string): void {
  mkdirSync(dirname(dirname(path)), { recursive: true });
  const work = mkdtempSync(join(dirname(dirname(path)), ".build-"));
  try {
    const staged = build(work);
    const publication = join(work, "publication");
    mkdirSync(publication);
    renameSync(staged, join(publication, basename(path)));
    try { renameSync(publication, dirname(path)); }
    catch (error) {
      if (!["EEXIST", "ENOTEMPTY"].includes((error as NodeJS.ErrnoException).code ?? "")) throw error;
    }
  } finally { rmSync(work, { recursive: true, force: true }); }
}
