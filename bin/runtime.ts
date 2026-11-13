import { spawnSync } from "node:child_process";
import { createHash } from "node:crypto";
import { existsSync, readdirSync } from "node:fs";
import { dirname, join, resolve } from "node:path";
import type { Snapshot } from "./snapshot";

const hash = (values: string[]) => createHash("sha256").update(values.sort().join("\n")).digest("hex");

/** The resolved runtime, not just link names or the compiler's dependencies. No ambient loader env. */
export function runtimeSnapshot(snapshot: Snapshot) {
  const captured = new Map<string, string>();
  let sharedCache: string[] | undefined;
  const inspect = (program: string, args: string[], env: Record<string, string>) => {
    const result = spawnSync(program, args, { env: { PATH: "/usr/bin:/bin", ...env }, encoding: "utf8" });
    if (result.status !== 0) throw new Error(`cannot resolve runtime dependencies: ${program} ${args.join(" ")}: ${result.stderr}`);
    return result.stdout;
  };
  const cachedSystemLibrary = (path: string): string[] => {
    if (!path.startsWith("/usr/lib/") && !path.startsWith("/System/Library/")) throw new Error(`unresolved runtime dependency: ${path}`);
    if (!sharedCache) {
      const root = ["/System/Volumes/Preboot/Cryptexes/OS/System/Library/dyld", "/System/Library/dyld"].find(existsSync);
      if (!root) throw new Error(`cannot locate dyld shared cache for ${path}`);
      const names = readdirSync(root);
      const base = (process.arch === "arm64" ? ["dyld_shared_cache_arm64e", "dyld_shared_cache_arm64"] : ["dyld_shared_cache_x86_64"]).find((name) => names.includes(name));
      if (!base) throw new Error(`cannot locate ${process.arch} dyld shared cache`);
      // System dylibs' entire transitive closure resides here. Bind every executable/data subcache.
      sharedCache = names.filter((name) => name === base || name.startsWith(`${base}.`) && !name.endsWith(".map") && !name.endsWith(".atlas"))
        .sort().map((name) => snapshot.file(join(root, name)));
      sharedCache.push(snapshot.file("/usr/lib/dyld"));
    }
    return sharedCache;
  };
  return (executable: string, env: Record<string, string>): string => {
    snapshot.assert();
    const key = `${executable}\n${JSON.stringify(env)}`;
    const old = captured.get(key);
    if (old) return old;
    const hashes = [snapshot.file(executable)];
    if (process.platform === "linux") {
      hashes.push(snapshot.optionalFile("/etc/ld.so.cache"), snapshot.optionalFile("/etc/ld.so.preload"));
      const linked = inspect("/usr/bin/ldd", [executable], env);
      if (linked.includes("not found")) throw new Error(`unresolved runtime dependency: ${linked}`);
      for (const match of linked.matchAll(/(?:=>\s+|^\s*)(\/[^\s]+)\s+\(/gm)) hashes.push(snapshot.file(match[1]));
    } else if (process.platform === "darwin") {
      const visited = new Set<string>();
      const rpaths = (file: string) => [...inspect("/usr/bin/otool", ["-l", file], env).matchAll(/cmd LC_RPATH\n\s*cmdsize \d+\n\s*path (.+?) \(offset/g)].map((m) => m[1]);
      const executableRpaths = rpaths(executable);
      const visit = (file: string) => {
        if (visited.has(file)) return;
        visited.add(file);
        const expand = (path: string) => path.replace("@loader_path", dirname(file)).replace("@executable_path", dirname(executable));
        const paths = [...rpaths(file), ...executableRpaths].map(expand);
        const linked = inspect("/usr/bin/otool", ["-L", file], env).split("\n").slice(1);
        for (const line of linked) {
          const dependency = line.trim().split(" (compatibility version")[0];
          if (!dependency) continue;
          let target = expand(dependency);
          if (target.startsWith("@rpath/")) {
            const candidates = paths.map((path) => resolve(path, target.slice(7)));
            target = candidates.find((path) => {
              if (existsSync(path)) return true;
              snapshot.optionalFile(path);
              return false;
            }) ?? "";
          }
          if (!target.startsWith("/")) throw new Error(`unresolved runtime dependency: ${dependency}`);
          if (existsSync(target)) { hashes.push(snapshot.file(target)); visit(target); }
          else hashes.push(...cachedSystemLibrary(target));
        }
      };
      visit(executable);
    } else throw new Error(`runtime dependency capture unsupported on ${process.platform}`);
    snapshot.assert();
    const identity = hash(hashes);
    captured.set(key, identity);
    return identity;
  };
}
