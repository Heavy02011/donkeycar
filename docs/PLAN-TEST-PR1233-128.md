# PLAN: Test von PR #1233 auf einem Donkeycar 1/28 (Legacy-Setup)

## Ziel
Diesen Plan verwenden, um **PR #1233** aus dem offiziellen Donkeycar-Repository auf einem **Donkeycar 1/28 (altes Setup / 1:28-Scale)** reproduzierbar zu testen.

- PR: https://github.com/autorope/donkeycar/pull/1233
- Referenz für 1/28 Legacy-Ansatz: https://www.diyrobocars.com/2026/01/06/running-donkeycar-on-a-128-scale-car/

## Testumfang
1. PR #1233 lässt sich auf dem Zielsystem sauber auschecken und installieren.
2. Basisfunktionen bleiben intakt (Start, Kamera-/Steuerloop, manuelles Fahren).
3. PR-betroffene Funktionen arbeiten wie erwartet (keine Regressionen).
4. Trainings-/Inference-Workflow bleibt funktionsfähig (soweit für 1/28 relevant).

## Voraussetzungen
- Donkeycar 1/28 gemäß Legacy-Dokumentation.
- Funktionierende Stromversorgung, Kamera, Lenkung und Throttle.
- Zugriff auf Donkeycar-Repo und passende Python-Umgebung.

## Testablauf
1. **Baseline ohne PR**
   - Commit/Config dokumentieren.
   - `manage.py drive` starten und manuelle Fahrt prüfen.
2. **PR #1233 auschecken**
   - PR-Branch holen und Dependencies aktualisieren.
3. **Start-/Buildtest**
   - Start ohne Exceptions, Parts geladen, Loop stabil.
4. **Funktionstest auf Fahrzeug**
   - Lenkung/Throttle testen, kurze Testfahrt aufzeichnen.
5. **Regressionstest**
   - Baseline vs. PR vergleichen und Auffälligkeiten notieren.
6. **Optional: Training/Inference-Sanity**
   - Kleine Trainings-/Inference-Probe mit Testdatensatz.

## Ergebnisvorlage
- Datum / Tester / Hardware-Variante
- Basis-Commit / PR-Commit
- Checkliste (Baseline, Starttest, Fahrt, Regression, optional Training)
- Beobachtungen und Entscheidung (Freigabe / mit Auflagen / nicht freigeben)
