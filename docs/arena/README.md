# 3D arena model (WRO 2026 rulebook dimensions)

`index.html` in this folder is an interactive three.js model of the WRO 2026 Future Engineers field. We use it to look at a legal random draw from any angle before we test a strategy on the mat, and to check how our 200 × 125 mm car fits the start zone and the 300 mm parking lot. It draws the field. It does not simulate driving and it does not score a run.

**Open it online:** https://blluewave.github.io/WRO-Future-Engineers-2026/docs/arena/

> **GitHub Pages must be enabled by the team before that link works.** In the repository go to *Settings > Pages > Build and deployment*, choose *Deploy from a branch*, branch `main`, folder `/ (root)`, and save. If the folder is set to `/docs` instead, the address becomes `https://blluewave.github.io/WRO-Future-Engineers-2026/arena/`.

**Open it locally:** open `docs/arena/index.html` in a desktop browser. The page loads three.js r128 from `cdnjs.cloudflare.com` and OrbitControls from `cdn.jsdelivr.net`, so it needs an internet connection. It remembers the last settings in the browser's local storage.

TODO(team): the control panel labels on the page are still in Arabic. Rule 7 (p.9) asks for all GitHub content in English for the international competition, so translate the labels before the scored commit.

## What the page does

| Control | What it changes | Rule it follows |
|---|---|---|
| Challenge | Open or Obstacle field | Section 8, p.11 and p.13 |
| New draw / seed | A random layout from a seed number, so the same layout can be rebuilt later | Draw procedures on p.12-16 |
| Driving direction | From the draw, or forced clockwise / counter-clockwise | Rule 9.3, p.17 |
| Open corridors | All 1000 mm, all 600 mm, or each straight drawn separately | Section 8, p.11-12 |
| Obstacle start | Inside the parking lot, or in the middle zone next to it | p.16 |
| 3D / Top view, walls, dimensions | Camera and overlays only | - |

For every draw the side panel lists the steps taken (coin tosses, die roll, cards drawn), the corridor widths, the inner block size, the start section, and whether our car footprint fits inside the start zone. A second table lists every field dimension with its rulebook page.

### Open Challenge draw (p.12-13)

1. Two coin tosses pick the starting section (Figure 7a, p.12).
2. Four coin tosses, clockwise from the start section, set each corridor: heads 1000 mm, tails 600 mm (p.12).
3. A die roll picks the start zone; the page rolls again if the zone falls inside a wall (Figure 7c, p.13).

When the four corridors differ, the inner walls form a rectangle that is no longer centred. That matches rule 13.16 (p.27).

### Obstacle Challenge draw (p.13-16)

1. Two coin tosses pick the section with a single traffic sign (Figure 8b, p.14).
2. One coin toss sets its colour: heads green, tails red (p.14).
3. Card 9 or card 10 is removed from the 36 cards, and one card is drawn per remaining straight, clockwise, without returning cards (Figure 8c, p.14-15).
4. Two more coin tosses pick the starting section, which always holds the parking lot. Signs in that section move to the row nearer the inner wall (Figures 8d and 8e, p.16).

## Every dimension and where it comes from

All page numbers refer to *WRO Future Engineers Category - Game Rules 2026* (55 pages).

| Item | Value | Rule | Page |
|---|---|---|---|
| Game mat | 3200 × 3200 mm (±5 mm) | 13.1 | 26 |
| Racetrack inner size | 3000 × 3000 mm (±5 mm) | 13.1 | 26 |
| Track colour | white | 13.2 | 26 |
| Exterior and interior wall height | 100 mm | 13.3, 13.5 | 26 |
| Wall thickness | not defined by the rules; the page draws 10 mm | 13.7 | 26 |
| Corner lines | 20 mm wide, orange CMYK (0, 60, 100, 0), blue CMYK (100, 80, 0, 0) | 13.9 | 26 |
| Corner line angle | 30 degrees | Figure 11 | 27 |
| Start zone dashed lines | 1 mm, CMYK (0, 0, 0, 30) | 13.10 | 26 |
| Start zone | 200 × 500 mm | 13.11 | 26 |
| Traffic sign seat | 50 × 50 mm, 1 mm line | 13.12, 13.13 | 26 |
| Sign evaluation circle | 85 mm diameter, 0.5 mm line, CMYK (20, 0, 100, 0) | 13.14, 13.15 | 26 |
| Seat rows across a 1000 mm corridor | 400 mm / 200 mm / 400 mm from the inner wall | Figure 11 | 27 |
| Seats per straight | 4 T-intersections and 2 X-intersections | Section 5, Figure 3 | 7 |
| Centre artwork square | 800 mm (the page shows the Blue Wave logo there) | Figure 11 | 27 |
| Open corridor width | 1000 mm or 600 mm (±100 mm at the International Final) | Section 8 | 11 |
| Obstacle corridor width | always 1000 mm (±10 mm at the International Final) | Section 8 | 13 |
| Traffic sign (pillar) | 50 × 50 × 100 mm | 13.19 | 28 |
| Pillars per colour | up to 7 | 13.20 | 28 |
| Red / green pillar colour | RGB (238, 39, 55) / RGB (68, 214, 44) | 13.21, 13.22 | 28 |
| Meaning of the colours | red: keep to the right side of the lane; green: keep to the left | Section 5 | 6 |
| Parking lot limiter | 200 × 20 × 100 mm, magenta RGB (255, 0, 255) | 13.25, 13.27 | 28 |
| Parking lot width | 200 mm | Section 5 | 8 |
| Parking lot length | 1.5 × robot length; 300 mm for our 200 mm car | Section 5 | 8 |
| Parking lot position | in the starting section, right limiter next to the dotted line | Section 5, Figure 4 | 8 |
| Car placement | footprint fully inside the start zone | 9.7 | 17 |
| Car orientation | front axle toward the next corner in the driving direction | 9.8 | 17 |
| Round length | 3 minutes for each challenge | 9.1, 9.2 | 17 |

With our car the lot leaves (300 - 200) / 2 = 50 mm at each end when the car is centred, and 200 - 125 = 75 mm across the lot. The panel shows both numbers for the current draw.

## What is not taken from the rulebook

- **Wall thickness.** Rule 13.7 leaves it undefined. The page uses 10 mm.
- **How the direction is drawn.** Rule 9.3 only says the direction is random. The page tosses a coin.
- **Die numbering in the other three sections.** Figure 7c (p.13) draws one section. The page rotates that numbering for the others.
- **Seat of the single sign.** The page places it in the middle column, outer row (the positions on cards 9 and 10). The rules text does not state it.
- **Our car.** The 200 × 125 mm footprint is the team's measurement from 14 September 2026. The 70 mm height is for display only; we have not measured the height.

## Related

- The main [README](../../README.md), section 8, explains how this page fits with our simulator and the Webots replays.
- The rulebook: https://wro-association.org/wp-content/uploads/WRO-2026-Future-Engineers-Self-Driving-Cars-General-Rules.pdf
