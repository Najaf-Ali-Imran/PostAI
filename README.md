<p align="center">
  <img src="assets/logo.png" alt="PostAI Logo" width="120" />
</p>

<h1 align="center">PostAI Logistics</h1>

<p align="center">
  <strong>Delivery Route Optimization — Running Entirely in Your Browser</strong>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/C++-17-blue?logo=cplusplus" alt="C++17" />
  <img src="https://img.shields.io/badge/WebAssembly-Enabled-blueviolet?logo=webassembly" alt="WASM" />
  <img src="https://img.shields.io/badge/Deploy-Vercel%20%7C%20Netlify-black?logo=vercel" alt="Deploy" />
  <img src="https://img.shields.io/badge/No%20Backend-Required-green" alt="No Backend" />
</p>

---

## 🚀 What is PostAI?

PostAI is an intelligent logistics route optimizer that calculates the **optimal delivery route** for multiple locations using advanced graph algorithms — all compiled to **WebAssembly** and running at near-native speed directly in the browser.

No server. No API calls. Just fast, offline-capable route optimization.

### ✨ Key Features

- **Optimal Route Calculation** — Held-Karp dynamic programming (exact TSP) for ≤20 locations
- **Scalable Approximation** — Christofides algorithm with 2-opt improvement for larger sets
- **Priority Deliveries** — High-priority locations are always visited first
- **Live Map Visualization** — Interactive Leaflet.js map with colored route segments
- **Truck Simulation** — Animated delivery truck following the optimized route
- **Fuel Calculator** — Estimate fuel consumption based on your vehicle's efficiency
- **ETA Estimation** — Calculated arrival times for each stop
- **Fully Offline** — Everything runs client-side via WebAssembly, no internet needed after load

---

## 🧠 Algorithms

| Algorithm | When Used | Guarantee |
|-----------|-----------|-----------|
| **Held-Karp DP** | ≤20 locations | Exact optimal solution |
| **Christofides** | >20 locations | Within 1.5× of optimal |
| **2-opt Local Search** | Always (post-processing) | Iterative improvement |
| **Haversine Distance** | All calculations | Accurate Earth-surface distances |
| **Priority-Constrained Routing** | When high-priority stops exist | High-priority stops visited first |

---

## 📁 Project Structure

```
PostAI/
├── index.html          # Landing page
├── app.html            # Main application (map + route optimizer)
├── documentation.html  # Project documentation
├── postai.js           # Generated WASM loader (Emscripten output)
├── postai.wasm         # Compiled C++ algorithms (WebAssembly binary)
├── postai_wasm.cpp     # C++ source — route optimization + Emscripten bindings
├── assets/
│   ├── logo.png        # Project logo
│   └── truck.png       # Delivery truck icon
└── README.md
```

---

## 🛠️ Building from Source

### Prerequisites

- [Emscripten SDK](https://emscripten.org/docs/getting_started/downloads.html)

### Compile

```bash
# Activate Emscripten
source ./emsdk/emsdk_env.sh     # Linux/Mac
# or: .\emsdk\emsdk_env.bat    # Windows

# Compile C++ to WebAssembly
emcc postai_wasm.cpp -o postai.js \
  --bind \
  -s WASM=1 \
  -s MODULARIZE=1 \
  -s EXPORT_NAME=createPostAIModule \
  -s ALLOW_MEMORY_GROWTH=1 \
  -s ENVIRONMENT=web \
  -s NO_EXIT_RUNTIME=1 \
  -O2 -std=c++17
```

### Run Locally

```bash
python -m http.server 8000
# Open http://localhost:8000/app.html
```

---



## 📸 How to Use

1. **Open the app** → click "Launch App" on the landing page
2. **Add a Warehouse** → click the warehouse button, then click on the map
3. **Add Houses** → click the house button, then click on delivery locations
4. **Toggle Priority** → mark stops as high-priority before adding
5. **Optimize Route** → click the optimize button — route is calculated instantly via WASM
6. **Simulate Delivery** → watch the truck follow the optimized route
7. **Undo** → remove the last added location

---

## ⚙️ Tech Stack

| Layer | Technology |
|-------|------------|
| Algorithms | C++17 (Held-Karp, Christofides, 2-opt) |
| Compilation | Emscripten → WebAssembly |
| Map | Leaflet.js + OpenStreetMap |
| Routing | Leaflet Routing Machine |
| Styling | Tailwind CSS |
| Deployment | Static files (Vercel / Netlify) |

---

## 📄 License

This project is for educational purposes — Najaf Ali University project.
