import { createHash } from "node:crypto";
import { closeSync, existsSync, openSync, readSync, readdirSync, readFileSync, statSync } from "node:fs";
import { join } from "node:path";

const hash = (data: string | Uint8Array) => createHash("sha256").update(data).digest("hex");
const stamp = (path: string) => {
  const s = statSync(path, { bigint: true });
  return `${s.dev}:${s.ino}:${s.size}:${s.mtimeNs}:${s.ctimeNs}`;
};

/** Content is read once, with a before/after check. Later checks also catch replacement and symlink retargeting. */
export class Snapshot {
  private readonly files = new Map<string, { stamp: string; hash: string }>();
  private readonly directories = new Map<string, string>();
  private failure: Error | undefined;
  private readonly absent = new Set<string>();
  optionalFile(path: string): string {
    if (existsSync(path)) return this.file(path);
    this.absent.add(path);
    return "absent";
  }
  file(path: string, normalize?: (text: string) => string): string {
    if (this.failure) throw this.failure;
    const old = this.files.get(path);
    if (old) {
      if (stamp(path) !== old.stamp) this.changed(path);
      return old.hash;
    }
    const before = stamp(path);
    let value: string;
    if (normalize) value = hash(normalize(readFileSync(path, "utf8")));
    else {
      // macOS system dylibs live in multi-GB shared-cache files, not standalone files.
      const fd = openSync(path, "r");
      try {
        const digest = createHash("sha256");
        const buffer = Buffer.allocUnsafe(4 * 1024 * 1024);
        for (let n = readSync(fd, buffer); n > 0; n = readSync(fd, buffer)) digest.update(buffer.subarray(0, n));
        value = digest.digest("hex");
      } finally { closeSync(fd); }
    }
    if (before !== stamp(path)) this.changed(path);
    this.files.set(path, { stamp: before, hash: value });
    return value;
  }
  tree(path: string): string {
    const entries = readdirSync(path).sort();
    this.directories.set(path, entries.join("\n"));
    return hash(entries.map((name) => {
      const child = join(path, name);
      return `${name}:${statSync(child).isDirectory() ? this.tree(child) : this.file(child)}`;
    }).join("\n"));
  }
  private changed(path: string): never {
    throw (this.failure ??= new Error(`oracle implementation changed: ${path}; restart the process`));
  }
  assert(): void {
    if (this.failure) throw this.failure;
    for (const path of this.absent) if (existsSync(path)) this.changed(path);
    for (const [path, old] of this.files) {
      try { if (stamp(path) !== old.stamp) this.changed(path); }
      catch { this.changed(path); }
    }
    for (const [path, entries] of this.directories) {
      try { if (readdirSync(path).sort().join("\n") !== entries) this.changed(path); }
      catch { this.changed(path); }
    }
  }
}
