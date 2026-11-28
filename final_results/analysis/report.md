# Final run: analysis

## Comparisons (blocks per game, second minus first)

- **M1** rule-based (B) vs random (A), n 100: 2.65 -> 6.14, diff +3.49 (95% CI +3.13 to +3.85); better/worse/same 95/1/4; Wilcoxon p 0.0000, Holm 0.0000: **difference**; early failures only in A: 22, only in B: 0
- **M2** rule-based + perception (C) vs rule-based (B), n 100: 6.14 -> 6.04, diff -0.10 (95% CI -0.29 to +0.09); better/worse/same 6/13/81; TOST +-0.5: p 0.0000 / 0.0000, 90% CI -0.26 to +0.06: **equivalent**; early failures only in B: 0, only in C: 0
- **M3** rule-based + probe (E) vs rule-based, varied friction (D), n 100: 5.88 -> 5.89, diff +0.01 (95% CI -0.16 to +0.18); better/worse/same 10/10/80; Wilcoxon p 0.9697, Holm 0.9697: **no difference found**; early failures only in D: 0, only in E: 0
- **M4** random + probe (G) vs random, varied friction (F), n 100: 2.61 -> 2.76, diff +0.15 (95% CI -0.03 to +0.33); better/worse/same 17/6/77; Wilcoxon p 0.0512, Holm 0.1025: **no difference found**; early failures only in F: 4, only in G: 3
- **S1** rule-based + probe on all blocks (H) vs rule-based, varied friction (D), n 100: 5.88 -> 6.03, diff +0.15 (95% CI -0.20 to +0.50); better/worse/same 29/25/46; Wilcoxon p 0.5216: **no difference found (secondary)**; early failures only in D: 0, only in H: 0

## Per condition

### A: random
- n 100, mean 2.65, median 2.5, SD 1.33; 6+ blocks 4; longest 6 (trial 512); early failures (<= 1 block) 22; middles 27% of turns; wall 6.6 min per game
- blocks per game: 1: 22, 2: 28, 3: 26, 4: 15, 5: 5, 6: 4
- how games end (outcome, method, layer, step): collapse end-pinch L4 pull x16; collapse end-pinch L2 pull x12; collapse end-pinch L3 pull x8; collapse end-pinch L5 pull x7; collapse end-pinch L1 pull x7; collapse push L3 regrab x6; collapse end-pinch L0 pull x5; collapse push L2 regrab x5

### B: rule-based
- n 100, mean 6.14, median 6.0, SD 1.46; 6+ blocks 76; longest 8 (trial 512); early failures (<= 1 block) 0; middles 12% of turns; wall 10.7 min per game
- blocks per game: 2: 1, 3: 1, 4: 20, 5: 2, 6: 35, 7: 19, 8: 22
- how games end (outcome, method, layer, step): collapse end-pinch L5 pull x48; collapse end-pinch L0 place x22; collapse end-pinch L4 pull x9; collapse end-pinch L3 pull x8; collapse end-pinch L0 pull x5; collapse push L3 pull x2; collapse end-pinch L1 place x2; collapse end-pinch L2 pull x2

### C: rule-based + perception
- n 100, mean 6.04, median 6.0, SD 1.51; 6+ blocks 70; longest 8 (trial 502); early failures (<= 1 block) 0; middles 12% of turns; wall 12.1 min per game
- blocks per game: 2: 1, 3: 1, 4: 22, 5: 6, 6: 32, 7: 15, 8: 23
- how games end (outcome, method, layer, step): collapse end-pinch L5 pull x45; collapse end-pinch L0 place x24; collapse end-pinch L3 pull x14; collapse end-pinch L4 pull x12; collapse push L3 push x2; collapse end-pinch L2 pull x2; collapse end-pinch L0 pull x1
- perception: 706 turns, worst block error median 0.79 mm, max 13.6 mm (height max 11.7 mm); over 0.875 mm in 211 turns; unseen block-turns 1

### D: rule-based, varied friction
- n 100, mean 5.88, median 6.0, SD 1.82; 6+ blocks 69; longest 8 (trial 500); early failures (<= 1 block) 2; middles 13% of turns; wall 11.0 min per game
- blocks per game: 0: 2, 2: 1, 3: 6, 4: 20, 5: 2, 6: 25, 7: 24, 8: 20
- how games end (outcome, method, layer, step): collapse end-pinch L5 pull x42; collapse end-pinch L0 place x19; collapse end-pinch L4 pull x12; collapse end-pinch L3 pull x12; collapse end-pinch L0 pull x6; collapse push L3 pedestal x3; collapse push L3 pull x2; collapse end-pinch L2 pull x2

### E: rule-based + probe
- n 100, mean 5.89, median 6.0, SD 1.84; 6+ blocks 70; longest 8 (trial 500); early failures (<= 1 block) 2; middles 11% of turns; wall 9.9 min per game
- blocks per game: 0: 2, 2: 1, 3: 7, 4: 18, 5: 2, 6: 29, 7: 18, 8: 23
- how games end (outcome, method, layer, step): collapse end-pinch L5 pull x40; collapse end-pinch L0 place x22; collapse end-pinch L3 pull x12; collapse end-pinch L4 pull x9; collapse push L3 pedestal x5; collapse end-pinch L0 pull x5; collapse end-pinch L4 place x2; collapse end-pinch L2 pull x2
- probe: 0.9 probes per game, 11 rejected, ~0 fallbacks, 0 games ended on a turn with a rejection; median N accepted edge nan (n 0) middle 3.08 (n 76), rejected edge nan (n 0) middle 5.58 (n 11)

### F: random, varied friction
- n 100, mean 2.61, median 3.0, SD 1.40; 6+ blocks 3; longest 7 (trial 558); early failures (<= 1 block) 24; middles 27% of turns; wall 6.6 min per game
- blocks per game: 0: 3, 1: 21, 2: 25, 3: 27, 4: 15, 5: 6, 6: 2, 7: 1
- how games end (outcome, method, layer, step): collapse end-pinch L2 pull x12; collapse end-pinch L4 pull x11; collapse end-pinch L5 pull x9; collapse end-pinch L3 pull x6; collapse push L3 regrab x6; collapse end-pinch L0 pull x6; collapse end-pinch L0 place x5; collapse push L3 pull x5

### G: random + probe
- n 100, mean 2.76, median 3.0, SD 1.46; 6+ blocks 3; longest 7 (trial 558); early failures (<= 1 block) 23; middles 21% of turns; wall 6.5 min per game
- blocks per game: 0: 3, 1: 20, 2: 22, 3: 24, 4: 19, 5: 9, 6: 2, 7: 1
- how games end (outcome, method, layer, step): collapse end-pinch L4 pull x20; collapse end-pinch L2 pull x17; collapse end-pinch L5 pull x11; collapse end-pinch L3 pull x7; collapse end-pinch L0 pull x5; collapse push L3 regrab x4; collapse end-pinch L1 pull x4; collapse push L3 pull x4
- probe: 1.1 probes per game, 33 rejected, ~0 fallbacks, 15 games ended on a turn with a rejection; median N accepted edge nan (n 0) middle 3.12 (n 80), rejected edge nan (n 0) middle 5.66 (n 33)

### H: rule-based + probe on all blocks
- n 100, mean 6.03, median 6.0, SD 1.89; 6+ blocks 72; longest 9 (trial 560); early failures (<= 1 block) 2; middles 14% of turns; wall 16.7 min per game
- blocks per game: 0: 2, 2: 2, 3: 7, 4: 14, 5: 3, 6: 23, 7: 24, 8: 24, 9: 1
- how games end (outcome, method, layer, step): collapse end-pinch L5 pull x47; collapse end-pinch L0 place x16; collapse end-pinch L4 pull x13; collapse end-pinch L3 pull x9; collapse push L3 pedestal x3; collapse end-pinch L5 place x2; collapse end-pinch L4 place x2; collapse end-pinch L0 pull x2
- probe: 8.6 probes per game, 161 rejected, ~12 fallbacks, 5 games ended on a turn with a rejection; median N accepted edge 2.16 (n 603) middle 3.21 (n 96), rejected edge 4.81 (n 144) middle 5.61 (n 17)
