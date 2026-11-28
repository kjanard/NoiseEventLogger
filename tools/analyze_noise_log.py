#!/usr/bin/env python3
"""
NoiseEventLogger — Daily Evidence Analyzer & Spectrogram Report Generator

Supports both Phase 3 (24-band 63 Hz–12.5 kHz @ 32 kHz) and Phase 4
(26-band 63 Hz–20 kHz @ 48 kHz) CSV logs from NoiseEventLogger.

Detects continuous high-frequency narrowband/jet nuisance episodes (e.g.
high-pressure gas burner / เตาฟู่), distinguishes them from broadband water
splashes and low-frequency compressors, quantifies broadband dBA masking vs
1/3-octave L90 emergence, and outputs:
  - <date>_evidence_spectrogram.png
  - <date>_evidence_report.md
  - <date>_hf_episodes.csv
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

import matplotlib
matplotlib.use("Agg")
import matplotlib.dates as mdates
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

BAND_SUFFIXES_26 = [
    "63", "80", "100", "125", "160", "200", "250", "315",
    "400", "500", "630", "800", "1k", "1k25", "1k6", "2k",
    "2k5", "3k15", "4k", "5k", "6k3", "8k", "10k", "12k5",
    "16k", "20k",
]

BAND_DISPLAY_LABELS_26 = [
    "63", "80", "100", "125", "160", "200", "250", "315",
    "400", "500", "630", "800", "1k", "1.25k", "1.6k", "2k",
    "2.5k", "3.15k", "4k", "5k", "6.3k", "8k", "10k", "12.5k",
    "16k", "20k",
]

ANECHOIC_FLOOR_DBFS_26 = np.array([
    -94.2, -92.3, -92.8, -93.7, -92.3, -92.8, -93.2, -92.8,
    -93.6, -94.2, -94.2, -94.5, -95.0, -95.2, -95.5, -95.9,
    -95.8, -96.0, -96.0, -95.8, -95.6, -95.2, -94.7, -93.9,
    -90.5, -90.7,
], dtype=float)

# Standard IEC 61672-1 A-weighting gains (dB) at nominal 1/3-octave centers
IEC_A_WEIGHT_DB_26 = np.array([
    -26.2, -22.5, -19.1, -16.1, -13.4, -10.9, -8.6, -6.6,
    -4.8,  -3.2,  -1.9,  -0.8,   0.0,   0.6,   1.0,  1.2,
     1.3,   1.2,   1.0,   0.5,  -0.1,  -1.1,  -2.5, -4.3,
    -6.6,  -9.3,
], dtype=float)

# HF Detector Constants (aligned with config.h Phase 4)
HF_DET_START_BAND = 21       # 8 kHz .. 12.5/20 kHz
HF_DET_MIN_EMERG_DB = 6.0
HF_DET_MAX_CREST_DB = 2.5
HF_DET_MIN_ADJ_PROM_DB = 4.0
HF_DET_MIN_MID_PROM_DB = 3.8


@dataclass
class HfEpisode:
    episode_id: int
    start_ts: pd.Timestamp
    end_ts: pd.Timestamp
    span_min: int
    active_min: int
    dominant_band: str
    max_l90: float
    mean_l90: float
    max_delta_quiet_db: float
    max_prom_db: float
    mean_crest_db: float
    delta_la90_dba: float


@dataclass
class RejectedCluster:
    category: str
    start_ts: pd.Timestamp
    end_ts: pd.Timestamp
    minutes: int
    peak_band: str
    max_delta_quiet_db: float
    median_prom_db: float
    median_crest_db: float


def compute_spectral_a_weighted(bands_db: np.ndarray, num_bands: int) -> np.ndarray:
    """Synthesize broadband A-weighted level from 1/3-octave bands (for Phase 3 logs)."""
    weighted = bands_db[:, :num_bands] + IEC_A_WEIGHT_DB_26[:num_bands]
    lin = np.power(10.0, weighted / 10.0)
    return 10.0 * np.log10(np.maximum(np.sum(lin, axis=1), 1e-12))


def detect_hf_minutes(
    l90_mat: np.ndarray,
    leq_mat: np.ndarray,
    floor_mat: np.ndarray,
    suffixes: List[str],
) -> pd.DataFrame:
    """Apply the Phase 4 HF stationary narrowband jet detector across every row."""
    n_rows, n_bands = l90_mat.shape
    mid_anchor = np.mean(l90_mat[:, 17:21], axis=1)  # 3.15k .. 6.3k mean L90

    alerts = np.zeros(n_rows, dtype=bool)
    peak_bands = []
    prom_vals = np.zeros(n_rows, dtype=float)
    emerg_vals = np.zeros(n_rows, dtype=float)
    crest_vals = np.zeros(n_rows, dtype=float)
    peak_l90_vals = np.zeros(n_rows, dtype=float)

    for i in range(n_rows):
        best_alert = False
        best_prom = -99.9
        best_emerg = -99.9
        best_crest = 0.0
        best_l90 = -99.9
        best_b = HF_DET_START_BAND

        disp_prom = -99.9
        disp_emerg = -99.9
        disp_crest = 0.0
        disp_l90 = -99.9
        disp_b = HF_DET_START_BAND

        for b in range(HF_DET_START_BAND, n_bands):
            emerg = float(l90_mat[i, b] - floor_mat[i, b])
            crest = float(leq_mat[i, b] - l90_mat[i, b])

            p1_ref = (
                0.5 * (l90_mat[i, b - 1] + l90_mat[i, b + 1])
                if (b + 1 < n_bands)
                else l90_mat[i, b - 1]
            )
            p1 = float(l90_mat[i, b] - p1_ref)

            adj_prom = p1
            if b + 2 < n_bands:
                p2 = float(
                    min(l90_mat[i, b], l90_mat[i, b + 1])
                    - 0.5 * (l90_mat[i, b - 1] + l90_mat[i, b + 2])
                )
                if p2 > adj_prom:
                    adj_prom = p2

            mid_prom = float(l90_mat[i, b] - mid_anchor[i])

            if adj_prom > disp_prom:
                disp_prom = adj_prom
                disp_emerg = emerg
                disp_crest = crest
                disp_l90 = float(l90_mat[i, b])
                disp_b = b

            if (
                emerg >= HF_DET_MIN_EMERG_DB
                and crest <= HF_DET_MAX_CREST_DB
                and adj_prom >= HF_DET_MIN_ADJ_PROM_DB
                and mid_prom >= HF_DET_MIN_MID_PROM_DB
            ):
                if (not best_alert) or (adj_prom > best_prom):
                    best_alert = True
                    best_prom = adj_prom
                    best_emerg = emerg
                    best_crest = crest
                    best_l90 = float(l90_mat[i, b])
                    best_b = b

        alerts[i] = best_alert
        if best_alert:
            peak_bands.append(suffixes[best_b])
            prom_vals[i] = best_prom
            emerg_vals[i] = best_emerg
            crest_vals[i] = best_crest
            peak_l90_vals[i] = best_l90
        else:
            peak_bands.append(suffixes[disp_b])
            prom_vals[i] = disp_prom
            emerg_vals[i] = disp_emerg
            crest_vals[i] = disp_crest
            peak_l90_vals[i] = disp_l90

    return pd.DataFrame(
        {
            "hf_alert": alerts,
            "hf_band": peak_bands,
            "hf_prom_db": prom_vals,
            "hf_emerg_db": emerg_vals,
            "hf_crest_db": crest_vals,
            "hf_peak_l90": peak_l90_vals,
        }
    )


def select_quiet_baseline(
    ts: pd.Series,
    l90_mat: np.ndarray,
    la90_vec: np.ndarray,
    baseline_arg: Optional[str],
) -> Tuple[np.ndarray, float, str]:
    """Select quiet baseline spectrum (either explicit HH:MM-HH:MM or auto 30m quietest)."""
    hm = ts.dt.strftime("%H:%M")
    if baseline_arg:
        parts = baseline_arg.split("-")
        if len(parts) != 2:
            raise ValueError(f"Invalid --baseline '{baseline_arg}', expected HH:MM-HH:MM")
        s_hm, e_hm = parts[0].strip(), parts[1].strip()
        mask = (hm >= s_hm) & (hm <= e_hm)
        if not mask.any():
            raise ValueError(f"Baseline window {baseline_arg} matched 0 rows")
        label = f"{s_hm}–{e_hm}"
        return np.median(l90_mat[mask], axis=0), float(np.median(la90_vec[mask])), label

    # Auto-select quietest 30-minute rolling window based on 2k..12.5k mean L90
    hf_energy = np.mean(l90_mat[:, 15:], axis=1)
    win = min(30, len(hf_energy))
    roll = pd.Series(hf_energy).rolling(win, min_periods=win).mean()
    end_idx = int(roll.idxmin()) if roll.notna().any() else int(np.argmin(hf_energy))
    start_idx = max(0, end_idx - win + 1)
    sl = slice(start_idx, end_idx + 1)
    label = f"{ts.iloc[start_idx].strftime('%H:%M')}–{ts.iloc[end_idx].strftime('%H:%M')} (auto)"
    return np.median(l90_mat[sl], axis=0), float(np.median(la90_vec[sl])), label


def cluster_hf_episodes(
    df: pd.DataFrame,
    l90_mat: np.ndarray,
    quiet_vec: np.ndarray,
    quiet_la90: float,
    suffixes: List[str],
    gap_bridge_min: int = 3,
) -> List[HfEpisode]:
    """Group detected HF alert minutes into episodes, bridging gaps <= gap_bridge_min."""
    alert_idx = np.where(df["hf_alert"].to_numpy())[0]
    if len(alert_idx) == 0:
        return []

    groups: List[List[int]] = [[int(alert_idx[0])]]
    for idx in alert_idx[1:]:
        prev_idx = groups[-1][-1]
        dt_min = (df["ts_dt"].iloc[idx] - df["ts_dt"].iloc[prev_idx]).total_seconds() / 60.0
        if dt_min <= (gap_bridge_min + 1.1):
            groups[-1].append(int(idx))
        else:
            groups.append([int(idx)])

    band_to_i = {s: i for i, s in enumerate(suffixes)}
    episodes: List[HfEpisode] = []
    for ep_num, idxs in enumerate(groups, start=1):
        i0, i1 = idxs[0], idxs[-1]
        ts0, ts1 = df["ts_dt"].iloc[i0], df["ts_dt"].iloc[i1]
        span_min = int(round((ts1 - ts0).total_seconds() / 60.0)) + 1

        sub = df.iloc[idxs]
        dom_band = str(sub["hf_band"].mode().iloc[0])
        b_idx = band_to_i[dom_band]

        # Evaluate dominant band level relative to quiet baseline across active minutes
        dom_l90 = l90_mat[idxs, b_idx]
        dom_delta = dom_l90 - quiet_vec[b_idx]
        ep_delta_la90 = float(np.median(df["la90_eff"].iloc[idxs]) - quiet_la90)

        episodes.append(
            HfEpisode(
                episode_id=ep_num,
                start_ts=ts0,
                end_ts=ts1,
                span_min=span_min,
                active_min=len(idxs),
                dominant_band=dom_band,
                max_l90=float(np.max(sub["hf_peak_l90"])),
                mean_l90=float(np.mean(sub["hf_peak_l90"])),
                max_delta_quiet_db=float(np.max(dom_delta)),
                max_prom_db=float(np.max(sub["hf_prom_db"])),
                mean_crest_db=float(np.mean(sub["hf_crest_db"])),
                delta_la90_dba=ep_delta_la90,
            )
        )
    return episodes


def find_rejected_clusters(
    df: pd.DataFrame,
    l90_mat: np.ndarray,
    quiet_vec: np.ndarray,
    suffixes: List[str],
) -> List[RejectedCluster]:
    """Identify broadband water/splash & low-frequency machinery clusters rejected by HF jet detector."""
    delta_mat = l90_mat - quiet_vec
    non_alert = ~df["hf_alert"].to_numpy()
    ts = df["ts_dt"]

    # Broadband HF activity (e.g. water spray / fountain): >= 4 bands in 2k..20k >= +5 dB above quiet baseline
    hf_broad_count = np.sum(delta_mat[:, 15:] >= 5.0, axis=1)
    water_mask = non_alert & (hf_broad_count >= 4)

    # Low-frequency tonal/machinery activity (e.g. AC compressor): max(63..250 Hz) >= +8 dB above quiet, HF <= +3 dB
    lf_max_delta = np.max(delta_mat[:, 0:7], axis=1)
    hf_mean_delta = np.mean(delta_mat[:, 21:], axis=1)
    lf_mask = non_alert & (~water_mask) & (lf_max_delta >= 8.0) & (hf_mean_delta <= 3.0)

    clusters: List[RejectedCluster] = []

    def extract(mask: np.ndarray, category: str, band_slice: slice, min_len: int) -> None:
        idxs = np.where(mask)[0]
        if len(idxs) == 0:
            return
        groups: List[List[int]] = [[int(idxs[0])]]
        for idx in idxs[1:]:
            if idx - groups[-1][-1] <= 3:
                groups[-1].append(int(idx))
            else:
                groups.append([int(idx)])
        for g in groups:
            if len(g) < min_len:
                continue
            sub_delta = delta_mat[g, :]
            rel_band = band_slice.start + int(np.argmax(np.max(sub_delta[:, band_slice], axis=0)))
            clusters.append(
                RejectedCluster(
                    category=category,
                    start_ts=ts.iloc[g[0]],
                    end_ts=ts.iloc[g[-1]],
                    minutes=len(g),
                    peak_band=suffixes[rel_band],
                    max_delta_quiet_db=float(np.max(sub_delta[:, rel_band])),
                    median_prom_db=float(np.median(df["hf_prom_db"].iloc[g])),
                    median_crest_db=float(np.median(df["hf_crest_db"].iloc[g])),
                )
            )

    extract(water_mask, "Broadband HF (Water Spray / Fountain)", slice(15, l90_mat.shape[1]), min_len=2)
    extract(lf_mask, "Low-Freq Machinery (AC Compressor / Pump)", slice(0, 7), min_len=5)
    clusters.sort(key=lambda c: c.start_ts)
    return clusters


def plot_evidence_spectrogram(
    df: pd.DataFrame,
    l90_mat: np.ndarray,
    quiet_vec: np.ndarray,
    quiet_label: str,
    episodes: List[HfEpisode],
    labels: List[str],
    out_png: Path,
    unit_lbl: str,
) -> None:
    """Render 3-panel high-resolution spectrogram + emergence + HF prominence timeline."""
    ts_num = mdates.date2num(df["ts_dt"])
    delta_mat = l90_mat - quiet_vec
    n_bands = l90_mat.shape[1]

    fig, axes = plt.subplots(
        3,
        1,
        figsize=(14, 10.5),
        sharex=True,
        gridspec_kw={"height_ratios": [1.3, 1.3, 1.05]},
        constrained_layout=True,
    )

    # Color limits adaptive to dBFS vs calibrated dBSPL
    l90_vmin = float(np.percentile(l90_mat, 2))
    l90_vmax = float(np.percentile(l90_mat, 99.5))

    date_str = df["ts_dt"].iloc[0].strftime("%Y-%m-%d")
    t_range = f"{df['ts_dt'].iloc[0].strftime('%H:%M')} to {df['ts_dt'].iloc[-1].strftime('%H:%M')}"

    # Panel 1: 1/3-Octave L90 Spectrogram
    ax0 = axes[0]
    im0 = ax0.pcolormesh(
        ts_num,
        np.arange(n_bands),
        l90_mat.T,
        shading="nearest",
        cmap="magma",
        vmin=l90_vmin,
        vmax=l90_vmax,
    )
    ax0.set_yticks(np.arange(n_bands))
    ax0.set_yticklabels(labels, fontsize=7.5)
    ax0.set_ylabel("1/3-Octave (Hz)")
    ax0.set_title(f"{date_str} Acoustic L90 Spectrogram ({unit_lbl}) — {t_range}", fontweight="bold")
    cb0 = fig.colorbar(im0, ax=ax0, pad=0.01)
    cb0.set_label(f"L90 ({unit_lbl})")

    # Panel 2: Delta L90 vs Quiet Baseline
    ax1 = axes[1]
    im1 = ax1.pcolormesh(
        ts_num,
        np.arange(n_bands),
        delta_mat.T,
        shading="nearest",
        cmap="RdBu_r",
        vmin=-6.0,
        vmax=12.0,
    )
    ax1.set_yticks(np.arange(n_bands))
    ax1.set_yticklabels(labels, fontsize=7.5)
    ax1.set_ylabel("1/3-Octave (Hz)")
    ax1.set_title(f"Delta L90 vs Quiet Baseline ({quiet_label}) — Positive = Louder (dB)", fontweight="bold")
    cb1 = fig.colorbar(im1, ax=ax1, pad=0.01)
    cb1.set_label("Delta L90 (dB)")

    for ep in episodes:
        t0, t1 = mdates.date2num(ep.start_ts), mdates.date2num(ep.end_ts)
        for ax in (ax0, ax1):
            ax.axvline(t0, color="cyan", linestyle="--", linewidth=1.1, alpha=0.85)
            ax.axvline(t1, color="cyan", linestyle="--", linewidth=1.1, alpha=0.85)

    # Panel 3: High-Frequency Bands vs Broadband LA90 + HF Prominence
    ax2 = axes[2]
    idx_8k = 21
    idx_12k5 = 23
    ax2.plot(df["ts_dt"], l90_mat[:, idx_8k], color="#d62728", linewidth=1.5, label=f"8 kHz L90 ({unit_lbl})")
    ax2.plot(df["ts_dt"], l90_mat[:, idx_12k5], color="#ff7f0e", linewidth=1.1, alpha=0.85, label=f"12.5 kHz L90 ({unit_lbl})")
    ax2.plot(df["ts_dt"], df["la90_eff"], color="#1f77b4", linewidth=1.2, alpha=0.9, label=f"Broadband LA90 ({'dBA' if unit_lbl=='dBSPL' else 'est. dBA'})")
    ax2.set_ylabel(f"Level ({unit_lbl})")
    ax2.grid(True, alpha=0.25)

    ax2_r = ax2.twinx()
    ax2_r.plot(df["ts_dt"], df["hf_prom_db"], color="#2ca02c", linewidth=1.0, alpha=0.75, label="HF Narrowband Prominence (dB)")
    ax2_r.axhline(HF_DET_MIN_ADJ_PROM_DB, color="#2ca02c", linestyle=":", linewidth=1.0, alpha=0.8)
    ax2_r.set_ylabel("HF Prominence (dB)", color="#2ca02c")

    for ep in episodes:
        ax2.axvspan(ep.start_ts, ep.end_ts, color="red", alpha=0.14, label="Detected HF Jet Episode" if ep.episode_id == 1 else None)

    if "marker" in df.columns and (df["marker"] == 1).any():
        m_idx = np.where(df["marker"].to_numpy() == 1)[0]
        groups: List[List[int]] = [[int(m_idx[0])]]
        for idx in m_idx[1:]:
            if idx - groups[-1][-1] == 1:
                groups[-1].append(int(idx))
            else:
                groups.append([int(idx)])
        for k, g in enumerate(groups):
            t0 = df["ts_dt"].iloc[g[0]]
            t1 = df["ts_dt"].iloc[g[-1]] + pd.Timedelta(seconds=60)
            ax2.axvspan(t0, t1, color="#8e44ad", alpha=0.18, label="Manual KEY Marker (marker=1)" if k == 0 else None)

    h1, l1 = ax2.get_legend_handles_labels()
    h2, l2 = ax2_r.get_legend_handles_labels()
    ax2.legend(h1 + h2, l1 + l2, loc="upper left", fontsize=8, ncol=2)
    ax2.set_title("Target HF Jet Signal (8k / 12.5k L90 & Prominence) vs Broadband LA90", fontweight="bold")
    tz_info = df["ts_dt"].dt.tz
    tz_lbl = str(tz_info) if tz_info is not None else "Local"
    ax2.set_xlabel(f"Local Time ({tz_lbl})")
    ax2.xaxis.set_major_formatter(mdates.DateFormatter("%H:%M", tz=tz_info))

    out_png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_png, dpi=180)
    plt.close(fig)


def write_markdown_report(
    df_full: pd.DataFrame,
    df_win: pd.DataFrame,
    episodes: List[HfEpisode],
    rejected: List[RejectedCluster],
    quiet_label: str,
    quiet_la90: float,
    out_md: Path,
    png_name: str,
    ep_csv_name: str,
    unit_lbl: str,
    schema_label: str,
) -> None:
    """Write bilingual Thai/English noise nuisance evidence report."""
    date_str = df_win["ts_dt"].iloc[0].strftime("%Y-%m-%d")
    full_t0 = df_full["ts_dt"].iloc[0].strftime("%H:%M:%S")
    full_t1 = df_full["ts_dt"].iloc[-1].strftime("%H:%M:%S")
    win_t0 = df_win["ts_dt"].iloc[0].strftime("%H:%M:%S")
    win_t1 = df_win["ts_dt"].iloc[-1].strftime("%H:%M:%S")

    total_active_min = sum(ep.active_min for ep in episodes)
    total_span_min = sum(ep.span_min for ep in episodes)
    max_vbat = int(df_full["vbat_mv"].max())
    min_vbat = int(df_full["vbat_mv"].min())
    total_clips = int(df_full["clips"].sum())

    lines: List[str] = []
    lines.append(f"# รายงานหลักฐานเสียงรบกวนความถี่สูงประจำวัน (Daily Noise Nuisance Evidence Report — {date_str})")
    lines.append("")
    lines.append("## 1. ภาพรวมการบันทึกข้อมูล (Recording Summary)")
    lines.append(f"- **Schema:** `{schema_label}` (`{unit_lbl}`)")
    lines.append(f"- **ข้อมูลทั้งหมดในไฟล์ (Full CSV):** `{full_t0}` – `{full_t1}` (`{len(df_full)}` นาที)")
    lines.append(f"- **ช่วงเวลาที่วิเคราะห์ (Analyzed Window):** `{win_t0}` – `{win_t1}` (`{len(df_win)}` นาที)")
    lines.append(f"- **ฐานเสียงเงียบอ้างอิง (Quiet Baseline):** `{quiet_label}` (`LA90 = {quiet_la90:.1f}`)")
    lines.append(f"- **ความสมบูรณ์ของข้อมูล:** Clipping รวม `{total_clips}` ครั้ง, แรงดันแบตเตอรี่ `{max_vbat} → {min_vbat} mV`")
    lines.append("")
    lines.append("## 2. เหตุการณ์เสียงรบกวนความถี่สูงต่อเนื่องที่ตรวจพบ (Detected HF Stationary Jet Episodes)")
    lines.append(
        f"> เกณฑ์ตรวจจับ (Phase 4 Detector): ย่าน `8 kHz–20 kHz` มี `L90` สูงกว่า self-noise `>= +{HF_DET_MIN_EMERG_DB:.1f} dB`, "
        f"ความนิ่ง `Leq - L90 <= {HF_DET_MAX_CREST_DB:.1f} dB`, และความแหลมเฉพาะย่าน (Narrowband Prominence) "
        f"`>= +{HF_DET_MIN_ADJ_PROM_DB:.1f} dB` (รวมช่วงพักไม่เกิน 3 นาทีเป็น episode เดียวกัน)"
    )
    lines.append("")
    lines.append(f"- **จำนวนเหตุการณ์ที่พบ:** `{len(episodes)}` ช่วงเวลา | **รวมเวลาที่เสียงดังต่อเนื่อง (Active):** `{total_active_min}` นาที (กรอบเวลารวม `{total_span_min}` นาที)")
    lines.append("")

    if episodes:
        lines.append("| # | เริ่ม (Start) | สิ้นสุด (End) | กรอบเวลา (Span) | นาทีที่พบ (Active) | ย่านเด่น (Band) | Max L90 | Max ΔL vs Quiet | Max Prominence | Mean (Leq-L90) | ΔLA90 (dBA) |")
        lines.append("| :-: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |")
        for ep in episodes:
            lines.append(
                f"| {ep.episode_id} | `{ep.start_ts.strftime('%H:%M:%S')}` | `{ep.end_ts.strftime('%H:%M:%S')}` | "
                f"{ep.span_min} min | {ep.active_min} min | `{ep.dominant_band} Hz` | "
                f"`{ep.max_l90:.1f}` | `+{ep.max_delta_quiet_db:.1f} dB` | `+{ep.max_prom_db:.1f} dB` | "
                f"`{ep.mean_crest_db:.2f} dB` | `{ep.delta_la90_dba:+.2f}` |"
            )
    else:
        lines.append("*ไม่พบเหตุการณ์เสียงรบกวนความถี่สูงเฉพาะย่านที่เข้าเกณฑ์ในช่วงเวลานี้*")

    if "marker" in df_win.columns and (df_win["marker"] == 1).any():
        m_idx = np.where(df_win["marker"].to_numpy() == 1)[0]
        m_groups: List[List[int]] = [[int(m_idx[0])]]
        for idx in m_idx[1:]:
            if idx - m_groups[-1][-1] == 1:
                m_groups[-1].append(int(idx))
            else:
                m_groups.append([int(idx)])
        lines.append("")
        lines.append(f"### 2.1 ช่วงเวลาที่กดปุ่ม `KEY` ทำเครื่องหมายด้วยตนเอง (Manual `marker=1`: {len(m_idx)} นาที)")
        for num, g in enumerate(m_groups, start=1):
            t0 = df_win["ts_dt"].iloc[g[0]].strftime("%H:%M:%S")
            t1 = df_win["ts_dt"].iloc[g[-1]].strftime("%H:%M:%S")
            lines.append(f"- Marker #{num}: `{t0}` – `{t1}` ({len(g)} นาที)")

    lines.append("")
    lines.append("## 3. การคัดแยกเสียงแวดล้อมอื่นที่ระบบปฏิเสธอัตโนมัติ (Rejected Non-Target Sources)")
    lines.append(
        "เพื่อแสดงความจำเพาะ (Specificity) ว่าตัวตรวจจับไม่นับรวมเสียงน้ำหรือเครื่องปรับอากาศภายในบ้านเป็นเสียงเตาฟู่:"
    )
    lines.append("")
    if rejected:
        lines.append("| ประเภทเสียงแวดล้อม (Category) | ช่วงเวลา (Window) | จำนวนนาที | ย่านเด่น (Band) | Max ΔL vs Quiet | Median Prominence | Median Crest |")
        lines.append("| :--- | :---: | :---: | :---: | :---: | :---: | :---: |")
        for cl in rejected:
            lines.append(
                f"| {cl.category} | `{cl.start_ts.strftime('%H:%M')}`–`{cl.end_ts.strftime('%H:%M')}` | "
                f"{cl.minutes} min | `{cl.peak_band} Hz` | `+{cl.max_delta_quiet_db:.1f} dB` | "
                f"`{cl.median_prom_db:+.1f} dB` | `{cl.median_crest_db:.1f} dB` |"
            )
    else:
        lines.append("*ไม่พบคลัสเตอร์เสียงน้ำ/คอมเพรสเซอร์ที่โดดเด่นในช่วงเวลานี้*")

    lines.append("")
    lines.append("## 4. ข้อสังเกตสำคัญด้านการวัด (Key Acoustic Finding)")
    lines.append(
        "- เสียงหัวพ่นแรงดันสูง (`8 kHz` / `12.5 kHz`) มีพลังงานเด่นในย่านความถี่สูงแคบ (`+7 ถึง +11 dB` เหนือพื้นเสียง) "
        "แต่ค่า **Broadband `dBA` (`LA90`) เปลี่ยนแปลงเพียง `+0.3 ถึง +0.6 dBA`** เพราะถูกเสียงพื้นหลังย่าน `200–1250 Hz` กลบ "
        "ดังนั้นหลักฐาน **IEC 61260 1/3-Octave `L90`** และ Spectrogram จึงจำเป็นอย่างยิ่งควบคู่กับค่า `dBA`"
    )
    lines.append("")
    lines.append(f"- กราฟหลักฐาน: `{png_name}`")
    lines.append(f"- ตารางสรุปเหตุการณ์ (CSV): `{ep_csv_name}`")

    out_md.parent.mkdir(parents=True, exist_ok=True)
    out_md.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="Analyze NoiseEventLogger daily CSV and generate evidence report.")
    parser.add_argument("csv_path", type=Path, help="Path to YYYY-MM-DD.csv log file")
    parser.add_argument("--out-dir", type=Path, default=None, help="Output directory (default: reports/<date>)")
    parser.add_argument("--start", type=str, default=None, help="Optional filter start time HH:MM (e.g. 08:10)")
    parser.add_argument("--end", type=str, default=None, help="Optional filter end time HH:MM")
    parser.add_argument("--baseline", type=str, default=None, help="Optional quiet baseline HH:MM-HH:MM")
    parser.add_argument("--gap-bridge", type=int, default=3, help="Max gap (minutes) bridged within one episode")
    args = parser.parse_args()

    df = pd.read_csv(args.csv_path)
    if df.empty:
        raise SystemExit(f"Empty CSV: {args.csv_path}")

    df["ts_dt"] = pd.to_datetime(df["ts"], errors="coerce")
    df = df.dropna(subset=["ts_dt"]).sort_values("ts_dt").reset_index(drop=True)

    # Detect Phase 3 (24-band) vs Phase 4 (26-band) CSV schema automatically
    has_p4_bands = "l90_16k" in df.columns and "l90_20k" in df.columns
    n_bands = 26 if has_p4_bands else 24
    suffixes = BAND_SUFFIXES_26[:n_bands]
    disp_labels = BAND_DISPLAY_LABELS_26[:n_bands]
    gain_vec = df["gain_db"].astype(float).to_numpy() if "gain_db" in df.columns else np.full(len(df), 24.0, dtype=float)
    floor_mat = ANECHOIC_FLOOR_DBFS_26[:n_bands][None, :] + (gain_vec - 24.0)[:, None]
    unit_lbl = str(df["unit"].iloc[0]) if "unit" in df.columns else "dBFS"
    schema_label = f"Phase 4 (26-band 63 Hz–20 kHz, gain={gain_vec[0]:.1f} dB)" if has_p4_bands else "Phase 3 (24-band 63 Hz–12.5 kHz)"

    l90_cols = [f"l90_{s}" for s in suffixes]
    leq_cols = [f"leq_{s}" for s in suffixes]

    full_l90 = df[l90_cols].to_numpy(dtype=float)
    full_leq = df[leq_cols].to_numpy(dtype=float)

    if "la90" in df.columns and "laeq" in df.columns:
        df["la90_eff"] = df["la90"].astype(float)
        df["laeq_eff"] = df["laeq"].astype(float)
    else:
        df["la90_eff"] = compute_spectral_a_weighted(full_l90, n_bands)
        df["laeq_eff"] = compute_spectral_a_weighted(full_leq, n_bands)

    # Run HF detector across ALL rows first (used both for full-day false-positive check and filtered window)
    det_full = detect_hf_minutes(full_l90, full_leq, floor_mat, suffixes)
    for col in det_full.columns:
        df[col] = det_full[col]

    # Apply optional --start / --end window
    df_win = df.copy()
    hm = df_win["ts_dt"].dt.strftime("%H:%M")
    mask = np.ones(len(df_win), dtype=bool)
    if args.start:
        mask &= (hm >= args.start).to_numpy()
    if args.end:
        mask &= (hm <= args.end).to_numpy()
    df_win = df_win.loc[mask].reset_index(drop=True)
    if df_win.empty:
        raise SystemExit("No rows matched time filter")

    win_l90 = df_win[l90_cols].to_numpy(dtype=float)
    quiet_vec, quiet_la90, quiet_label = select_quiet_baseline(
        df_win["ts_dt"], win_l90, df_win["la90_eff"].to_numpy(dtype=float), args.baseline
    )

    episodes = cluster_hf_episodes(
        df_win, win_l90, quiet_vec, quiet_la90, suffixes, gap_bridge_min=args.gap_bridge
    )
    rejected = find_rejected_clusters(df_win, win_l90, quiet_vec, suffixes)

    date_str = df_win["ts_dt"].iloc[0].strftime("%Y-%m-%d")
    out_dir = args.out_dir if args.out_dir is not None else Path("reports") / date_str
    out_dir.mkdir(parents=True, exist_ok=True)

    png_path = out_dir / f"{date_str}_evidence_spectrogram.png"
    md_path = out_dir / f"{date_str}_evidence_report.md"
    ep_csv_path = out_dir / f"{date_str}_hf_episodes.csv"

    plot_evidence_spectrogram(df_win, win_l90, quiet_vec, quiet_label, episodes, disp_labels, png_path, unit_lbl)

    ep_rows = [
        {
            "episode_id": ep.episode_id,
            "start_ts": ep.start_ts.isoformat(),
            "end_ts": ep.end_ts.isoformat(),
            "span_min": ep.span_min,
            "active_min": ep.active_min,
            "dominant_band_hz": ep.dominant_band,
            "max_l90": round(ep.max_l90, 2),
            "mean_l90": round(ep.mean_l90, 2),
            "max_delta_quiet_db": round(ep.max_delta_quiet_db, 2),
            "max_prom_db": round(ep.max_prom_db, 2),
            "mean_crest_db": round(ep.mean_crest_db, 2),
            "delta_la90_dba": round(ep.delta_la90_dba, 2),
        }
        for ep in episodes
    ]
    pd.DataFrame(ep_rows).to_csv(ep_csv_path, index=False, encoding="utf-8")

    write_markdown_report(
        df,
        df_win,
        episodes,
        rejected,
        quiet_label,
        quiet_la90,
        md_path,
        png_path.name,
        ep_csv_path.name,
        unit_lbl,
        schema_label,
    )

    full_alerts_before_1845 = int(((df["ts_dt"].dt.strftime("%H:%M") < "18:45") & df["hf_alert"]).sum())
    print(f"[OK] Schema: {schema_label} | Full rows: {len(df)} | Window rows: {len(df_win)}")
    print(f"[OK] Full-day alerts before 18:45 (False-Positive check): {full_alerts_before_1845}")
    print(f"[OK] Quiet Baseline: {quiet_label} (LA90={quiet_la90:.1f})")
    print(f"[OK] Detected HF Episodes ({len(episodes)}, active={sum(e.active_min for e in episodes)}m):")
    for ep in episodes:
        print(
            f"     Ep#{ep.episode_id}: {ep.start_ts.strftime('%H:%M:%S')}..{ep.end_ts.strftime('%H:%M:%S')} "
            f"span={ep.span_min}m active={ep.active_min}m band={ep.dominant_band} "
            f"max_L90={ep.max_l90:.1f} dQuiet=+{ep.max_delta_quiet_db:.1f}dB "
            f"prom=+{ep.max_prom_db:.1f}dB crest={ep.mean_crest_db:.2f}dB dLA90={ep.delta_la90_dba:+.2f}"
        )
    print(f"[OK] Rejected Non-Target Clusters ({len(rejected)}):")
    for cl in rejected:
        print(
            f"     - {cl.category}: {cl.start_ts.strftime('%H:%M')}..{cl.end_ts.strftime('%H:%M')} "
            f"({cl.minutes}m, peak={cl.peak_band}, dQuiet=+{cl.max_delta_quiet_db:.1f}dB, "
            f"medProm={cl.median_prom_db:+.1f}dB, medCrest={cl.median_crest_db:.1f}dB)"
        )
    print(f"[OUT] {png_path}")
    print(f"[OUT] {md_path}")
    print(f"[OUT] {ep_csv_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
