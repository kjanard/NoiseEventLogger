#pragma once

#include <Arduino.h>
#include "config.h"
#include "dsp_octave.h"

// ============================================================================
// Phase 4: NVS-Persisted SPL Calibration & Non-Blocking Serial CLI
// ============================================================================

bool Calibration_Init();

float Calibration_GetOffsetDb();
float Calibration_GetBandOffsetDb(uint8_t band_idx);
bool Calibration_IsCalibrated();
const char *Calibration_GetUnitLabel();

/**
 * Process non-blocking Serial calibration commands on Core 0:
 *   CAL HELP
 *   CAL STATUS
 *   CAL <ref_spl_db>              (e.g. CAL 94.0 with 1 kHz calibrator tone)
 *   CAL OFFSET <offset_db>
 *   CAL BAND <idx_or_label> <db>  (e.g. CAL BAND 8k -1.2)
 *   CAL RESET
 */
void Calibration_PollSerial(const SecondReport_t *latest_sec, bool has_sec);
