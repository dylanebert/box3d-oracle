// The statistics a step's spread rests on: the normal distribution's tails and quantiles, the
// chi-square quantile the Centered Normality Hypothesis bound inflates a sample deviation by, and the
// Anderson-Darling test of normality that decides whether that bound applies.

/** erfc(x), Numerical Recipes' Chebyshev fit (erfcc), fractional error below 1.2e-7 everywhere. */
export function erfc(x: number): number {
  const z = Math.abs(x);
  const t = 1 / (1 + 0.5 * z);
  const r = t * Math.exp(-z * z - 1.26551223 + t * (1.00002368 + t * (0.37409196 + t * (0.09678418 + t * (-0.18628806 + t * (0.27886807 + t * (-1.13520398 + t * (1.48851587 + t * (-0.82215223 + t * 0.17087277)))))))));
  return x >= 0 ? r : 2 - r;
}

/** The standard normal distribution function. */
export const normalCdf = (x: number) => 0.5 * erfc(-x / Math.SQRT2);

/** The standard normal quantile, Acklam's rational approximation (relative error below 1.2e-9). */
export function normalQuantile(p: number): number {
  if (!(p > 0 && p < 1)) throw new Error(`normal quantile of ${p}`);
  const a = [-3.969683028665376e1, 2.209460984245205e2, -2.759285104469687e2, 1.38357751867269e2, -3.066479806614716e1, 2.506628277459239];
  const b = [-5.447609879822406e1, 1.615858368580409e2, -1.556989798598866e2, 6.680131188771972e1, -1.328068155288572e1];
  const c = [-7.784894002430293e-3, -3.223964580411365e-1, -2.400758277161838, -2.549732539343734, 4.374664141464968, 2.938163982698783];
  const d = [7.784695709041462e-3, 3.224671290700398e-1, 2.445134137142996, 3.754408661907416];
  const tail = (q: number) => (((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q + c[4]) * q + c[5]) / ((((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q + 1);
  if (p < 0.02425) return tail(Math.sqrt(-2 * Math.log(p)));
  if (p > 1 - 0.02425) return -tail(Math.sqrt(-2 * Math.log(1 - p)));
  const q = p - 0.5, r = q * q;
  return ((((((a[0] * r + a[1]) * r + a[2]) * r + a[3]) * r + a[4]) * r + a[5]) * q) / (((((b[0] * r + b[1]) * r + b[2]) * r + b[3]) * r + b[4]) * r + 1);
}

function lnGamma(x: number): number {
  const g = [76.18009172947146, -86.50532032941677, 24.01409824083091, -1.231739572450155, 0.1208650973866179e-2, -0.5395239384953e-5];
  let y = x, tmp = x + 5.5, ser = 1.000000000190015;
  tmp -= (x + 0.5) * Math.log(tmp);
  for (const coefficient of g) ser += coefficient / ++y;
  return -tmp + Math.log((2.5066282746310005 * ser) / x);
}

/** The regularized lower incomplete gamma function P(a, x) (Numerical Recipes' gammp). */
function gammaP(a: number, x: number): number {
  if (x <= 0) return 0;
  const front = Math.exp(-x + a * Math.log(x) - lnGamma(a));
  if (x < a + 1) {
    let ap = a, sum = 1 / a, del = sum;
    for (let n = 0; n < 10_000 && Math.abs(del) > Math.abs(sum) * 1e-15; n++) sum += del *= x / ++ap;
    return sum * front;
  }
  let b = x + 1 - a, c = 1e300, d = 1 / b, h = d;
  for (let i = 1; i < 10_000; i++) {
    const an = -i * (i - a);
    b += 2;
    d = an * d + b;
    if (Math.abs(d) < 1e-300) d = 1e-300;
    c = b + an / c;
    if (Math.abs(c) < 1e-300) c = 1e-300;
    d = 1 / d;
    const del = d * c;
    h *= del;
    if (Math.abs(del - 1) < 1e-15) break;
  }
  return 1 - front * h;
}

/** The p-quantile of the chi-square distribution with `dof` degrees of freedom, by bisection. */
export function chiSquareQuantile(p: number, dof: number): number {
  let lo = 0, hi = Math.max(1, dof);
  while (gammaP(dof / 2, hi / 2) < p) hi *= 2;
  for (let i = 0; i < 200; i++) {
    const mid = (lo + hi) / 2;
    if (gammaP(dof / 2, mid / 2) < p) lo = mid;
    else hi = mid;
  }
  return (lo + hi) / 2;
}

/** Anderson-Darling's A*², the statistic modified for a mean and variance estimated from the sample
 * (D'Agostino and Stephens, Goodness-of-Fit Techniques, 1986, table 4.7). `undefined` when the sample
 * has no spread. */
export function andersonDarling(sample: readonly number[]): number | undefined {
  const n = sample.length;
  const mean = sample.reduce((s, x) => s + x, 0) / n;
  const sd = Math.sqrt(sample.reduce((s, x) => s + (x - mean) ** 2, 0) / (n - 1));
  if (!(sd > 0)) return undefined;
  const z = sample.map((x) => (x - mean) / sd).sort((a, b) => a - b);
  const lnCdf = (x: number) => Math.log(Math.max(normalCdf(x), 1e-300));
  let s = 0;
  for (let i = 0; i < n; i++) s += (2 * i + 1) * (lnCdf(z[i]) + lnCdf(-z[n - 1 - i]));
  return (-n - s / n) * (1 + 0.75 / n + 2.25 / (n * n));
}

/** A*²'s critical value at significance 0.05, mean and variance estimated (the same table). */
export const ANDERSON_DARLING_05 = 0.752;
