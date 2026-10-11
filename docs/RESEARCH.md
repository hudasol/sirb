# Sirb: Phase 0 research

Retrieved 2026-10-11 by web search and page fetches. Items marked **UNVERIFIED** were not confirmed on a retrieved page. Some pages were read through a summarizing fetch tool, so exact figures should be re-checked against the linked source before being quoted elsewhere. Nothing here is a compliance claim.

## 1. Prior art and what Sirb adds

### 1.1 Open-source tools

| Tool | Role | License (as read from GitHub web pages) | Source |
|---|---|---|---|
| Skybrush Studio for Blender | design and validation of shows | GPL-3.0 | https://github.com/skybrush-io |
| Skybrush Live / Server / Viewer | ground control, server, viewer | GPL-3.0 | https://github.com/skybrush-io/live |
| Crazyswarm | nano-quadrotor swarm | MIT (stated on page), no longer maintained | https://github.com/USC-ACTLab/crazyswarm |
| Crazyswarm2 | ROS 2 stack for Crazyflie | MIT (stated on page) | https://github.com/IMRCLab/crazyswarm2 |

Skybrush Studio Server license: UNVERIFIED (page failed to fetch). The LICENSE files were not read directly.

**Skybrush validation (documented).** The Blender Safety Check panel validates proximity (centre-to-centre minimum distance), altitude, horizontal and vertical speed, acceleration and yaw rate, and charts closest-pair distance. https://docs.skybrush.io/public/skybrush-studio-for-blender/latest/panels/safety_and_export/safety_check.html. Studio 4.2 adds yaw-rate validation and a validation report (https://skybrush.io/blog/2026-03-29-skybrush-studio-4.2/). The FAQ describes preprogrammed paths "validated beforehand", a dual-layer geofence and a bubble geofence (https://docs.skybrush.io/public/faq/latest/security.html).

### 1.2 Commercial tools (public claims only)

Verge Aero states that its design studio "simulates and renders every show", that the software "mathematically validates the flight paths", and describes onboard GNSS gating (3D fix, at least 6 satellites, PDOP of 2.5 or less) and geofences (https://docs.verge.aero/drone-show-technology/safety, https://www.verge.aero/essential-guide-to-drone-show-safety). "Independent" is used there for hardware estimators, not for software verification. These are vendor claims, not independently checked. Drone Show Software makes only generic safety claims (https://www.droneshowsoftware.com). Ehang: not searched.

### 1.3 Academic work (retrieved pages)

| Work | Relevance |
|---|---|
| Mellinger and Kumar, "Minimum snap trajectory generation and control for quadrotors", ICRA 2011 (https://ieeexplore.ieee.org/abstract/document/5980409; author list not in fetched text, from general knowledge) | polynomial trajectories |
| Augugliaro, Schoellig, D'Andrea, "Generation of collision-free trajectories for a quadrocopter fleet: a sequential convex programming approach", IROS 2012 (https://www.dynsyslab.org/wp-content/papercite-data/pdf/augugliaro-iros12.pdf) | SCP, roughly 4 to 12 vehicles, no disturbance analysis |
| Morgan, Chung, Hadaegh, "Model predictive control of swarms of spacecraft using sequential convex programming", JGCD 37(6), 2014 (https://authors.library.caltech.edu/records/mvhns-j8a04) | decentralized MPC with SCP |
| van den Berg, Guy, Lin, Manocha, "Reciprocal n-body collision avoidance" (ORCA), Springer 2011 (https://www.doi.org/10.1007/978-3-642-19457-3_1; page not fully fetched, authors from general knowledge) | velocity-obstacle avoidance |
| Wang, Ames, Egerstedt, "Safety barrier certificates for collisions-free multirobot systems", IEEE T-RO 33(3), 2017 (https://authors.library.caltech.edu/records/tshzw-g4v69) | control barrier functions |
| Preiss, Honig, Sukhatme, Ayanian, "Crazyswarm", ICRA 2017 (https://act.usc.edu/crazyswarm.html) | swarm platform |
| Honig et al., "Trajectory planning for quadrotor swarms", IEEE T-RO (https://act.usc.edu/publications/Hoenig_TRO2018.pdf) | up to 200 robots in simulation; assignment by Threshold algorithm (makespan); position error used only for downwash radius |
| "Optimal waypoint assignment for designing drone light show formations" (CHungSDA), Results in Control and Optimization 2022 (https://doaj.org/article/7414ccfb73514b2ab457cf3ec8417ed3) | constrained Hungarian method |
| Moon et al., "Energy-efficient swarming flight formation transitions using the improved fair Hungarian algorithm", Sensors 21(4):1260, 2021 (https://www.mdpi.com/1424-8220/21/4/1260) | assignment, validated in a 100-drone show |
| Alqudsi, "Enhancing drone light shows performances: optimal allocation and trajectories", arXiv 2603.24401, 2026 (https://arxiv.org/abs/2603.24401) | unified assignment and trajectory; claims 1,008 drones in about 1 s |
| Park, Mok, Au, "Robust evacuation for multi-drone failure in drone light shows", AAAI 2026 (https://arxiv.org/html/2601.06728v1) | failure evacuation; explicitly does not model GNSS error |

### 1.4 Honest assessment of the differentiation claim

**Mostly true, with caveats.** In what was retrieved: Skybrush validates the nominal plan only; Verge validates inside its own toolchain and publishes no degradation sweep; Augugliaro assumes adequate tracking; Honig uses position error only for downwash sizing; Park et al. omit GNSS error. A two-query search for drone-show GNSS-error papers found nothing specific.

Caveats: runtime GNSS handling exists (Verge gating, Skybrush bubble geofence), so the gap is a pre-show, published, statistical analysis and an independent verifier, not GNSS awareness as such. Verge says it runs internal simulated-sensor tests; contents and results are UNVERIFIED and unpublished. Commercial internals are closed, so "none do it" cannot be shown.

**Defensible pitch wording:** "We found no open-source drone-show tool that publishes separation margins under modelled GNSS error, and none that ships a verifier independent of the planner."

Gaps not closed: Skybrush firmware GNSS failsafe docs, Ehang, wider scholarly search ("light show", "RTK error", "separation"), Mellinger-Kumar author list, ORCA venue confirmation, Honig TRO year.

## 2. Real-world failure modes (retrieved)

### 2.1 Official findings

| Date, place | Outcome | Stated cause | Source |
|---|---|---|---|
| 21 Dec 2024, Lake Eola, Orlando | about 500 drones, collisions, one child hit | NTSB final (DCA25LA065, 15 Sep 2026): probable cause failure to locate the show centre over the physical location of the aircraft; contributing: geofence contrary to company procedure, incorrect show heading, lack of oversight allowing individual errors to go undetected | https://data.ntsb.gov/carol-repgen/api/Aviation/ReportMain/GenerateNewestReport/199458/pdf |
| (same) preliminary details | five drones did not accept launch data, soft reboot, parameter file never sent, show rotated 7 degrees, FAA suspended waiver | secondary reporting of the preliminary report; outlets disagree on which way round the 1 m and 5 m geofence figures go | https://dronedj.com/2025/01/17/orlando-drone-show-ntsb-report/ |
| 14 Jul 2023, Docklands, Melbourne | 427 of 500 drones lost into the water | ATSB: wind exceeded published capability, pilot unaware (wind not clearly displayed, no exceedance alert), link to almost 400 drones lost | https://www.atsb.gov.au/node/12449 |
| 27 Oct 2018, Hong Kong | more than 40 of 100 drones fell | GPS signal interfered with, per organizer and Tourism Board statements (not a published investigation) | https://insidegnss.com/?p=178873 |

### 2.2 Unconfirmed or operator-only causes

SeaTac Jul 2024 (operator suspects interference, unconfirmed); Perth Nov 2022 (GPS and wind "considered"); Folly Beach Dec 2024 (wind, bystander opinion); Quanzhou Dec 2024 (battery or jamming speculated, UNVERIFIED); Shanghai Jun 2021 (unknown); Budva, Montenegro New Year 2026 (jammer suspected, GPS World 21 Jan 2026, https://www.gpsworld.com/jammer-suspected-as-cause-of-budva-drone-show-disaster/); Vivid Sydney May 2026 (operator cites radio frequency change). Several of these only exist in secondary trade press. No Dubai or UAE show incident and no GCAA investigation was found. No NTSB or FAA finding naming GNSS or RTK interference for any show was found.

### 2.3 Published safety guidance (as summarized by fetch)

- UK CAA CAP 722E (Dec 2022): crowd line at least 50 m from operational volume; automatic geofence and flight termination function; return-to-home "not an appropriate flight termination function for swarms" (https://caa.co.uk/publication/download/18053).
- Hong Kong CAD AC-010 (18 Jun 2026): 300 ft AGL, cordon at least 50 m, geofencing of all drones, procedures for C2 link loss, GNSS loss and low battery (https://www.cad.gov.hk/documents/AC-010.pdf).
- One FAA Part 107 waiver (a single operator's certificate, not general guidance): abort thresholds such as GPS/RTK lost for 1 minute holds and lands the show (https://www.faa.gov/sites/faa.gov/files/107W-2026-01914-Richard-Boss-CoW.pdf).
- EASA guidance, Malta swarm notice and UAE drone-show rules: not retrieved.

### 2.4 Failure category to stress-test design

| Category | Evidence quality | Sirb stress-test implication |
|---|---|---|
| Configuration, human, setup | best (NTSB, ATSB) | pre-flight consistency validator: centre and heading versus physical grid, geofence versus declared standard, crowd-line distance after any transform, fleet plan hash; perturbation tests (rotate, shift, mismatch two drones) the validator must catch |
| Wind | official (ATSB) | altitude-varying gust profiles, wind above the airframe limit, check formation error and separation |
| Comms | official (ATSB), operator claims | link-loss dropout and regional blackout, failsafe descent evaluation |
| GNSS | one official-side attribution (Hong Kong), others speculative | per-drone and regional dropout, drift, fix-to-float fallback, step offset; framed as one scenario among several |
| Battery | speculation only | endurance spread under wind, clustered low-battery failsafes |
| Collision and launch stage | Orlando, Melbourne | separate launch-lift stage test |
| Abort logic | Orlando, CAP 722E | hold, land and terminate with FAA-waiver-like count thresholds; measure abort completion time |

## 3. GNSS parameterization

| Item | Value | Source |
|---|---|---|
| ZED-F9P RTK horizontal accuracy | 0.01 m + 1 ppm CEP (1 km baseline, patch antennas) | u-blox ZED-F9P datasheet UBX-17051259 (https://www.mouser.com/datasheet/2/813/ZED_F9P_Data_Sheet-2489427.pdf) |
| RTK convergence | under 10 s multi-GNSS, under 30 s GPS only | same |
| Standard GPS SPS horizontal, vertical (95%) | 8 m, 13 m | https://gps.gov/systems/gps/performance |
| Measured F9P RTK fixed, open sky | CEP99 0.031 m horizontal | one user's forum test (https://discuss.ardupilot.org/t/cubepilot-here4-neo-f9p-vs-holybro-h-rtk-zed-f9p-sensors-comparison/112475) |
| Measured occluded RTK float | CEP50 0.578 m, CEP99 2.281 m | same forum test |
| Gulf interference | secondary-sourced counts and drift figures (RICS, as of 24 Mar 2026, unverified by RICS itself); aviation bulletin CAAS SIB 2026-01 | https://www.rics.org/news-insights/gnss-spoofing-and-mitigation-in-the-arabian-gulf, https://www.caas.gov.sg/docs/default-source/docs---srg/caas_sib_2026-01_middle_east_conflict_13-mar-2026.pdf |

The error model uses a first-order Gauss-Markov form (Joerger et al. 2020, https://www.aoe.vt.edu/content/dam/aoe_vt_edu/people/faculty/joerger/publications/Overbounding%20GNSS_INS%20Integration%20with%20Unknown%20Gauss-Markov%20Error%20Parameters%202020.pdf). That source is measurement-level (pseudorange), so **all position-domain values below are ASSUMPTIONS**, informed by the figures above:

| Parameter | Value | Status |
|---|---|---|
| RTK-fixed white noise sigma | 0.015 m | ASSUMPTION |
| RTK-float GM bias sigma, tau | 0.3 m, 30 s | ASSUMPTION |
| Standard GNSS GM bias sigma, tau | 3 m, 300 s | ASSUMPTION (3 m 1-sigma gives about 8 m at 95%, my calculation) |
| Fix-to-float fallback delay | 1 to 5 s | ASSUMPTION |
| Re-fix time | 10 to 30 s | floor from datasheet convergence figure |
| Spoofing pull-off rate | 0.1 to 5 m/s | ASSUMPTION |
| Dropout | seconds to minutes per injection | ASSUMPTION (RICS event length of 3 to 4 h is system level only) |

Not retrieved: RTK behavior under jamming (time to lose fix, fallback), UAV position-domain GM values, ArduPilot EKF noise defaults.

## 4. UAE context (background only)

Shows: 6,500 drones at the Sheikh Zayed Festival, Al Wathba, 31 Dec 2025 (Khaleej Times, 8 Dec 2025: https://www.khaleejtimes.com/uae/new-years-eve-62-minute-fireworks-6500-drones-light-up-al-wathba-2025); a Guinness phoenix formation in Ras Al Khaimah, 31 Dec 2025 (https://www.thenationalnews.com/lifestyle/2026/01/01/uae-new-years-eve-fireworks-world-records-drones/); 1,000 drones for a GCAA-hosted ICAO symposium dinner, Feb 2025; 600 drones at Global Village for National Day 2024 (https://mediaoffice.ae/en/news/2024/december/04-12/global-village-marks-the-uae-53rd-eid-al-etihad-with-a-historic-zayedandrashid). No vendor was named for the major shows.

Regulatory context, as read from GCAA CAR-UAS Issue 01 (issued 6 Feb 2018, applicable 4 Mar 2018; may not be the current issue, **UNVERIFIED**): scope includes "UA events" and "drone flight shows"; an operator authorisation and a UAS operational approval are required; 400 ft height limit; operations over or near crowds, in controlled airspace or within 8 km of airports need specific GCAA approval; the document has no swarm provisions. Source: https://www.gcaa.ae/en/PublishingImages1/CAR-UAS%20-%20UNMANNED%20AIRCRAFT%20SYSTEM%20(UAS)%20AND%20OPERATIONS-%20ISSUE%2001.pdf. The GCAA registration page describes a two-step permission process but no separate drone-show process (https://gcaa.gov.ae/en/Pages/UASRegistration.aspx). Dubai and Abu Dhabi primary pages and current no-fly maps were not verified. TDRA coordinates GNSS interference complaints per an ICAO MID working paper (https://www.icao.int/sites/default/files/MID/MeetingDocs/2026/FM%20WG5%20%26%20CNS%20SG15/3-%20FM%20WG5%20-%20Documentation/WP10.pdf).

**Sirb does not satisfy, implement or claim compliance with any of the above.** A real show needs an operator safety case and approvals from the relevant UAE authorities.

## 5. Vehicle and wind parameters

| Parameter | Value | Source |
|---|---|---|
| Verge Aero X7 mass | 830 g | https://www.verge.aero/EN-X7-Fact-Sheet-Verge-Aero-v.2026.03.pdf |
| X7 max speed horizontal, vertical | 10 m/s, 6 m/s | same |
| X7 show flight time | 16 min | same |
| X7 max sustained wind | 9 m/s | same |
| X7 thrust, drag, inertia | not published | ASSUMPTION |
| Crazyflie 2.1 mass | 29 g | https://www.bitcraze.io/documentation/hardware/crazyflie_2_1/crazyflie_2_1-datasheet.pdf |

Wind turbulence uses Dryden forms from MIL-HDBK-1797 as implemented in MathWorks documentation (https://www.mathworks.com/help/aeroblks/drydenwindturbulencemodelcontinuous.html): low-altitude sigma_w = 0.1 W20, scale lengths from altitude. These models were built for piloted aircraft; applying them to a 0.8 kg multirotor at 20 to 100 m is an **ASSUMPTION**.

## 6. Consequences for Sirb's design

1. Pitch GNSS stress testing as one scenario among several; the best-evidenced failure classes are configuration errors and wind.
2. Add a pre-flight consistency validator and perturbation tests (reversible scope addition, decided here).
3. Treat launch lift-off and abort handling as explicit stress-test stages.
4. Vehicle numbers: use X7 published values where available; all dynamics beyond them are labeled assumptions.
5. All GNSS position-domain parameters are assumptions, to be parameterized later from Tashwish outputs.
