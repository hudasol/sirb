# Hazard analysis (FMEA style)

Scope: which retrieved real-world causes of drone show incidents Sirb's checks and stress tests
do and do not address. Sirb is a simulation-first planner, verifier and stress tester. It is not
flight-certified and this document makes no regulatory-compliance claim. No severity numbers or
probabilities are given here: none come from code or a retrieved source, and Monte-Carlo
probabilities from the stress tests are properties of the simulated model, not real-world risk
(`docs/METHODS.md` section 9).

Every cause comes from `docs/RESEARCH.md` (retrieved 2026-10-11), cited as that file cites it,
with its own UNVERIFIED marks kept. Additional retrieval on 2026-10-12 is listed after the table.
`tests/test_hazards_doc.py` parses the table and fails if any file or code symbol named in the
"Covering check" column does not exist, so this list cannot silently drift. It checks existence
only, not that the check really covers the hazard; that judgement is in the Coverage column.

Coverage levels: **Covered by exact verifier** (closed-form check, exact under its stated point
model); **Covered by simulation under ASSUMED model** (Monte-Carlo with assumed parameters);
**Partially**; **Not covered**.

Stored stress outputs (`results/stress_*.json`) are produced by a separate job and are not part
of this branch; rows reference the experiment functions that generate them, not numbers.

## Table

| ID | Incident or cause (source) | Failure mode | Effect | Covering check | Coverage | Residual gap |
|---|---|---|---|---|---|---|
| H1 | Orlando 21 Dec 2024: failure to locate the show centre over the physical location (NTSB final DCA25LA065, https://data.ntsb.gov/carol-repgen/api/Aviation/ReportMain/GenerateNewestReport/199458/pdf) | Ground station holds a centre that differs from the declared or physical one | Whole show shifted toward crowd, geofence or other hazards | `sirb.preflight.preflight` (CENTER_MISMATCH, PLACED_PLAN_VIOLATES_LIMITS); `tests/test_preflight.py::test_orlando_style_perturbations_are_caught`; `tests/test_preflight.py::test_shift_toward_crowd_violates_after_placement` | Partially | Compares a declaration with what was loaded. If the declared centre is itself wrong (no survey tie to the physical site), nothing detects it. Survey procedure is out of scope. |
| H2 | Orlando (same NTSB final): contributing, geofence contrary to company procedure. Preliminary-report geofence figures disagree between outlets (https://dronedj.com/2025/01/17/orlando-drone-show-ntsb-report/) | Loaded geofence or buffer differs from the safety case | Aircraft allowed or contained in the wrong volume | `sirb.preflight.preflight` (GEOFENCE_BUFFER_MISMATCH, GEOFENCE_MISMATCH); `tests/test_preflight.py::test_orlando_style_perturbations_are_caught` | Partially | Only catches disagreement between two dicts. A company procedure that is not written into the declaration is not checked. No check of the geofence actually programmed in the aircraft firmware. |
| H3 | Orlando: incorrect show heading (NTSB final); preliminary reporting says the show was rotated 7 degrees (secondary, same dronedj URL) | Loaded heading differs from declared heading | Rotated show, crowd margin and geofence margin change | `sirb.preflight.preflight` (HEADING_MISMATCH); `sirb.preflight.transform_fleet`; `tests/test_preflight.py::test_transform_is_exact_rigid_motion` | Partially | Same as H1: the true physical heading is not observed. After a mismatch the plan is re-verified by the exact verifier under the loaded transform, but only for geofence, crowd and altitude. |
| H4 | Orlando: lack of oversight allowing individual errors to go undetected (NTSB final) | Human procedure, checklists, second-person verification absent or skipped | Any of H1 to H3, H5 passes unnoticed | none | Not covered | Human oversight and procedures cannot be modelled here. Preflight is a software aid and its INFO finding NOT_CHECKED says so. |
| H5 | Orlando preliminary details (secondary reporting): five drones did not accept launch data, parameter file never sent (dronedj URL above) | Some aircraft never receive or hold a different plan | Aircraft fly the wrong or no trajectory, collisions with neighbours | `sirb.preflight.fleet_plan_hashes`; `sirb.preflight.preflight` (PLAN_NOT_RECEIVED, PLAN_HASH_MISMATCH, AIRCRAFT_COUNT_MISMATCH); `tests/test_preflight.py::test_two_aircraft_swapped_plans_detected` | Partially | Exact hash comparison, but only against a record a ground station supplies. Sirb has no ground-station integration and cannot read an aircraft. Autopilot parameter files are not hashed. |
| H6 | Orlando preliminary (secondary): soft reboot of aircraft, software and parameter faults (dronedj URL above); no final-report finding on this | Show-control or autopilot software fault, reboot, wrong parameters | Unplanned behaviour of individual aircraft | none | Not covered | Show-control software and firmware are not modelled. Closest proxy is the generic single-aircraft failure events (see H12), which do not represent software behaviour. |
| H7 | Melbourne 14 Jul 2023: wind exceeded published capability, pilot unaware (ATSB, https://www.atsb.gov.au/node/12449) | Wind above the airframe limit | Loss of control of many aircraft | `sirb.preflight.preflight` (WIND_ABOVE_LIMIT, WIND_NEAR_LIMIT); `tests/test_preflight.py::test_wind_limits`; `sirb.disturbance.StressConfig.w20`; `sirb.disturbance.wind_velocity`; `scripts/run_stress.py::exp_sweeps` | Partially | Preflight compares a supplied forecast with a supplied limit. The stress model has no thrust, tilt or authority limit and perfect feed-forward (`docs/METHODS.md` section 5), so it can show small errors in winds where a real aircraft is beyond its limit. Dryden intensities and the 1/7 profile are assumptions for a small multirotor. No real-time wind display or exceedance alert exists (the ATSB finding about the pilot display is a procedure and ground-software matter). |
| H8 | Melbourne (ATSB, same URL): link to almost 400 drones lost | Command link loss to many aircraft at once | Aircraft fall or continue uncontrolled | `sirb.disturbance.StressConfig.comms_loss_fraction`; `sirb.disturbance.sample_events`; `sirb.contingency.apply_events`; `scripts/run_stress.py::exp_contingency` | Partially | Events are drawn per drone at independent uniform times, so a simultaneous fleet-wide link loss is not produced. Behaviours hold, descend, corridor and continue are simulated; link range, latency and RF environment are not modelled. |
| H9 | Hong Kong 27 Oct 2018: more than 40 of 100 drones fell; GPS interference per organizer and Tourism Board statements, not a published investigation (https://insidegnss.com/?p=178873) | GNSS degraded or lost across much of the fleet | Loss of position hold, collisions or falls | `sirb.disturbance.gnss_error`; `sirb.disturbance.StressConfig.gnss_deg_fraction`; `sirb.disturbance.StressConfig.gnss_loss_fraction`; `scripts/run_stress.py::exp_sweeps` | Covered by simulation under ASSUMED model | Per-drone errors are independent; interference is correlated across the fleet (`docs/METHODS.md` section 9, item 5) and the model cannot produce a shared pull-off. All position-domain GNSS parameters are assumptions (RESEARCH.md section 3). The cause itself is an attribution, not a finding. |
| H10 | Budva, Montenegro New Year 2026: jammer suspected, unconfirmed (GPS World 21 Jan 2026, https://www.gpsworld.com/jammer-suspected-as-cause-of-budva-drone-show-disaster/) | Loss of GNSS navigation shortly after launch | Aircraft returned, some fell into the sea, collisions with aircraft still taking off (per the same article, which attributes the analysis to a vendor and cites no official investigation) | `sirb.disturbance.gnss_error`; `sirb.stress.run_trial` | Partially | Fleet-correlated loss and the launch phase are not represented: dropout windows are placed at 15 to 85 percent of show time, per drone, independently (`gnss_error`). See H11, H13, H14. |
| H11 | Return-to-home failsafe: UK CAA CAP 722E says return-to-home is "not an appropriate flight termination function for swarms" (https://caa.co.uk/publication/download/18053, as summarized by fetch); Budva reports drones began returning (GPS World URL above) | Failsafe makes many aircraft converge on one point | Mid-air collisions among failsafing aircraft | none | Not covered | No return-to-home behaviour exists in `sirb.contingency`; the implemented behaviours are continue, hold, descend, corridor, rejoin and ballistic. Whether a given autopilot would return home is not modelled. |
| H12 | Falling or uncontrolled aircraft (Hong Kong, Melbourne, Orlando, Budva above). No motor-failure or battery root cause was retrieved. | Single aircraft loses control | Aircraft falls onto neighbours below | `sirb.disturbance.StressConfig.motor_failures`; `sirb.stress.ballistic_track`; `tests/test_stress.py::test_ballistic_fall_causes_violation_on_toy_column`; `scripts/run_stress.py::exp_sweeps` | Covered by simulation under ASSUMED model | Ballistic fall uses an assumed horizontal drag decay of 0.5 per second and ignores wind during the fall. Only distances between drones are evaluated. Failure arrival is uniform in time, not tied to any retrieved failure data. |
| H13 | Launch stage collisions (Orlando, Melbourne; RESEARCH.md section 2.4 recommends a separate launch-lift test) | Collision while the fleet lifts off or lands | Contact between aircraft on or just above the ground | `sirb.verify.verify`; `scripts/run_stress.py::family_show` | Partially | Exact verifier checks the nominal launch and landing legs for separation; the stress runs include launch as part of the whole show. No dedicated launch-lift stress test exists, and ground effect, takeoff sequencing errors and aircraft that fail to leave the ground are not modelled. |
| H14 | Falling aircraft strike people or property: Orlando, a child was hit (NTSB final, URL in H1); Hong Kong and Melbourne losses | Aircraft impact outside the show volume | Injury or damage on the ground | none | Not covered | Stress runs evaluate inter-drone distance only. Ground impact point, crowd distance and geofence under disturbance and after failure are not evaluated (`docs/METHODS.md` section 9, item 9). |
| H15 | Crowd line: UK CAP 722E, crowd at least 50 m from operational volume (URL in H11); Hong Kong AC-010, cordon at least 50 m (https://www.cad.gov.hk/documents/AC-010.pdf), both as summarized by fetch | Plan or placed plan reaches inside the crowd buffer | Spectators inside the operational volume | `sirb.verify.verify`; `sirb.spec.validate_show`; `sirb.preflight.preflight` (CROWD_MARGIN_AFTER_PLACEMENT) | Partially | Exact for the planned path against the declared straight crowd line. Spectator behaviour, people outside the declared line, real crowd positions and the 50 m figure being applicable to any particular jurisdiction are not covered. Not evaluated under disturbance. |
| H16 | Wind differential between neighbours (Melbourne; Dryden/MIL-HDBK-1797 per RESEARCH.md section 5) | Neighbouring aircraft pushed together by differing gusts | Separation loss without any failure | `sirb.disturbance.wind_velocity`; `tests/test_stress.py::test_wind_differential_between_neighbours_is_correlated`; `sirb.stress.monte_carlo` | Covered by simulation under ASSUMED model | Tracking loop is a generic second order assumption with no latency, saturation or attitude. Turbulence is first order Gauss-Markov with fixed correlation time, not a Dryden spectrum. |
| H17 | RTK float degradation: one forum test, occluded float CEP50 0.578 m and CEP99 2.281 m (RESEARCH.md section 3, https://discuss.ardupilot.org/t/cubepilot-here4-neo-f9p-vs-holybro-h-rtk-zed-f9p-sensors-comparison/112475) | Fix falls back to float, slowly varying position bias | Differential position error between aircraft | `sirb.disturbance.StressConfig.gnss_bias_sigma`; `sirb.disturbance.StressConfig.gnss_bias_tau`; `sirb.stress.required_value`; `scripts/run_stress.py::exp_spacing` | Covered by simulation under ASSUMED model | Sigma and tau are assumptions informed by one user's forum test. Real RTK errors are largely common to nearby aircraft, so independence overstates the differential (conservative for separation, METHODS section 5). Time to lose fix and fallback behaviour were not retrieved. |
| H18 | GNSS spoofing or carry-off in the Gulf (RICS, secondary; CAAS SIB 2026-01; RESEARCH.md section 3 URLs) | Slow common pull-off of position estimates | Shared drift, possibly undetected by aircraft | none | Not covered | The nearest code, `gnss_error` in `sirb/disturbance.py`, has no pull-off ramp and no cross-aircraft correlation. RESEARCH.md lists pull-off rates of 0.1 to 5 m/s as an assumption, not implemented. `docs/METHODS.md` states this is not conservative. |
| H19 | GNSS dropout and abort thresholds: one FAA Part 107 waiver, GPS/RTK lost for 1 minute holds and lands the show (https://www.faa.gov/sites/faa.gov/files/107W-2026-01914-Richard-Boss-CoW.pdf, a single operator's certificate); Hong Kong AC-010 requires procedures for GNSS loss | Per-aircraft GNSS loss, fleet-level abort rules | Aircraft lose position hold; fleet response needed | `sirb.disturbance.StressConfig.gnss_loss_s`; `sirb.disturbance.StressConfig.gnss_dead_reckoning_rate`; `sirb.contingency.rejoin_reference`; `scripts/run_stress.py::exp_contingency` | Partially | Dropout default is 10 s with an assumed dead-reckoning random walk. Fleet-level abort logic (count thresholds, hold and land, termination) is not implemented or tested (`docs/METHODS.md` section 9, item 10). The rejoin reference is optimistic unless `rejoin_vmax` is set. |
| H20 | Battery: Quanzhou Dec 2024, battery or jamming speculated, UNVERIFIED in RESEARCH.md; X7 flight time 16 min (datasheet, RESEARCH.md section 5) | Low battery, endurance spread, clustered low-battery failsafes | Aircraft descend or fall mid-show | `sirb.disturbance.StressConfig.battery_low_fraction`; `scripts/run_stress.py::exp_sweeps` | Partially | Only a discrete low-battery event at a random time with an assumed descend behaviour. No battery model: no capacity, wind-dependent consumption, voltage sag, cold or heat effects, or clustering. No retrieved source establishes battery as a root cause. |
| H21 | RF or radio interference other than GNSS: Vivid Sydney May 2026, operator cites radio frequency change; SeaTac Jul 2024, operator suspects interference, unconfirmed (both as listed in RESEARCH.md section 2.2, no URL recorded there) | Fleet-wide link or sensor disturbance from a shared RF cause | Correlated failure of many aircraft | none | Not covered | All stochastic events are independent per drone. Correlated RF interference across the fleet, and any frequency-planning issue, are outside the model. Evidence for both incidents is operator statements only. |
| H22 | Downwash and aerodynamic interaction: Honig et al., T-RO, uses position error for downwash sizing (https://act.usc.edu/publications/Hoenig_TRO2018.pdf, RESEARCH.md section 1.3). Not an incident source. | Aircraft flying through another's wake | Thrust and tracking loss, effectively closer approach than point model | none | Not covered | `sirb.verify.verify` treats drones as points and the stress contact check uses a sphere of radius 0.24 m; there is no vertical-offset or wake rule (`docs/NEXT_STEPS.md` item 7). The 2.0 m threshold is an assumption. |
| H23 | Planned trajectory itself conflicts (design basis, no retrieved incident) | Nominal plan violates separation, speed, acceleration, jerk, altitude or geofence limits | Collision or limit exceedance even with perfect tracking | `sirb.verify.verify`; `sirb.certificate.verify_certificate`; `tests/test_verify.py` | Covered by exact verifier | Exact up to floating point and root-finding tolerance for the plan as written, under the declared point model and the declared limits, which are assumptions where marked in `sirb/spec.py`. Says nothing about actual aircraft motion. |

## Additional retrieval, 2026-10-12

- Budva (GPS World, URL in H10): fetched and read. It reports about 600 drones launched, returning
  after about 15 seconds, some falling into the sea and some colliding with drones taking off, and
  the cause as a suspected jammer from a Croatian company's analysis relayed by a news outlet;
  explicitly not confirmed and no official investigation cited. This added the launch collision and
  return behaviour details used in H10, H11 and H13.
- SeaTac, July 2024 (Dronelife page named "55 drones fall out of the sky"): a fetch of the page
  returned no statement of any cause, so SeaTac stays as RESEARCH.md has it: operator suspects
  interference, unconfirmed.
- Two further searches for final reports of other drone show incidents surfaced only Orlando and
  Melbourne material already in RESEARCH.md plus press coverage. No new official finding was
  retrieved; nothing was added from search-result titles alone.

## Not covered

- H4: human oversight and procedures. Needed: an operator safety case and a documented, second-person
  checked procedure; Sirb can only supply machine-checkable artefacts for it.
- H6: show-control software, firmware and parameter handling. Needed: ground-station and autopilot
  integration or software-in-the-loop runs, plus parameter-file hashing.
- H11: return-to-home or other converging failsafes. Needed: a failsafe behaviour model per target
  autopilot, taken from its documentation, and a test with many aircraft failsafing together.
- H14: ground impact, people and property under disturbance and failure. Needed: impact-point
  evaluation of falling aircraft against geofence and crowd line, and a cited model of fall dynamics.
- H18: GNSS spoofing, common pull-off. Needed: a fleet-correlated drift model with cited rates and
  a stress scenario with a shared offset (see `docs/GNSS_PARAMS.md` for the spatial and temporal fields
  reserved for this).
- H21: RF interference correlated across the fleet. Needed: a shared-cause event model and
  retrieved link-level data.
- H22: downwash. Needed: a literature-cited wake exclusion rule in the verifier.
- Also outside every row: spectator behaviour, centre and heading surveys (the true physical values),
  and battery behaviour beyond a discrete event.

## Partial

- H1, H2, H3: consistency of declarations only; need a surveyed truth source and a firmware-level
  geofence readback.
- H5: needs a ground-station feed of per-aircraft plan hashes and parameter-file hashes.
- H7: needs an airframe authority limit in the tracking model, a calibrated tracking loop, and a
  wind exceedance alert design (operational).
- H8: needs a correlated fleet-wide link loss scenario and a link model.
- H10: needs fleet-correlated GNSS loss and a launch-phase scenario.
- H13: needs a dedicated launch-lift stress stage.
- H15: exact only for a declared line; does not evaluate disturbance or spectators.
- H19: needs fleet-level abort rules (hold, land, terminate with count thresholds) implemented and tested.
- H20: needs a battery model with capacity, wind-dependent draw and spread, calibrated to a datasheet or log.
