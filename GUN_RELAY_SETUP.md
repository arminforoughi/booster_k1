# Gun.js Relay Server Setup for K1 Fleet

**Purpose:** Distributed state management for multi-robot coordination
**Architecture:** Decentralized database with HTTP relay for ROS2 integration

---

## 🎯 Why Gun.js?

- ✅ **Decentralized:** No single point of failure
- ✅ **Real-time sync:** All robots see state changes instantly
- ✅ **Conflict resolution:** CRDT-based automatic merge
- ✅ **Offline-first:** Robots continue operating if relay is down
- ✅ **Peer-to-peer:** Robots can sync directly without server

---

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────┐
│         Gun.js Relay Server (Node.js)               │
│                                                      │
│  Gun Database:                                      │
│    /fleet/robots/{robot_id} → Heartbeat data       │
│    /fleet/tasks/{task_id}   → Task queue           │
│                                                      │
│  HTTP REST API:                                     │
│    GET  /fleet/robots       → All robot states     │
│    PUT  /fleet/robots/{id}  → Update robot         │
│    GET  /fleet/tasks        → All tasks            │
│    PUT  /fleet/tasks/{id}   → Create/claim task    │
└─────────────────────────────────────────────────────┘
           ↓         ↓         ↓
    ┌──────────┬──────────┬──────────┐
    │ Robot 001│ Robot 002│ Robot 003│
    │ (Python) │ (Python) │ (Python) │
    │ requests │ requests │ requests │
    └──────────┴──────────┴──────────┘
```

---

## 📦 Server Installation

### Option A: Docker (Recommended)

Create `gun-relay/Dockerfile`:
```dockerfile
FROM node:18-alpine

WORKDIR /app

# Install Gun.js
RUN npm install gun express cors body-parser

# Copy server code
COPY server.js .

EXPOSE 8765

CMD ["node", "server.js"]
```

Create `gun-relay/server.js`:
```javascript
const Gun = require('gun');
const express = require('express');
const cors = require('cors');
const bodyParser = require('body-parser');

const app = express();
const port = process.env.PORT || 8765;

// Middleware
app.use(cors());
app.use(bodyParser.json());

// Initialize Gun
const gun = Gun({
  web: app,
  file: 'data',  // Persist to disk
  localStorage: false,
  radisk: true
});

// Health check
app.get('/health', (req, res) => {
  res.json({ status: 'ok', timestamp: Date.now() });
});

// REST API for Python clients

// GET /fleet/robots - Get all robot states
app.get('/fleet/robots', async (req, res) => {
  const robots = {};

  gun.get('fleet').get('robots').map().once((data, id) => {
    if (data) robots[id] = data;
  });

  setTimeout(() => res.json(robots), 100);
});

// PUT /fleet/robots/:id - Update robot state
app.put('/fleet/robots/:id', (req, res) => {
  const robotId = req.params.id;
  const data = req.body;

  gun.get('fleet').get('robots').get(robotId).put(data);

  res.json({ success: true, robot_id: robotId });
});

// GET /fleet/tasks - Get all tasks
app.get('/fleet/tasks', async (req, res) => {
  const tasks = {};

  gun.get('fleet').get('tasks').map().once((data, id) => {
    if (data) tasks[id] = data;
  });

  setTimeout(() => res.json(tasks), 100);
});

// GET /fleet/tasks/:id - Get specific task
app.get('/fleet/tasks/:id', async (req, res) => {
  const taskId = req.params.id;

  gun.get('fleet').get('tasks').get(taskId).once((data) => {
    if (data) {
      res.json(data);
    } else {
      res.status(404).json({ error: 'Task not found' });
    }
  });
});

// PUT /fleet/tasks/:id - Create or update task
app.put('/fleet/tasks/:id', (req, res) => {
  const taskId = req.params.id;
  const data = req.body;

  gun.get('fleet').get('tasks').get(taskId).put(data);

  res.json({ success: true, task_id: taskId });
});

// PATCH /fleet/tasks/:id - Partial update (for task completion)
app.patch('/fleet/tasks/:id', (req, res) => {
  const taskId = req.params.id;
  const updates = req.body;

  gun.get('fleet').get('tasks').get(taskId).get('status').put(updates.status);
  gun.get('fleet').get('tasks').get(taskId).get('completed_by').put(updates.completed_by);
  gun.get('fleet').get('tasks').get(taskId).get('completed_at').put(updates.completed_at);
  gun.get('fleet').get('tasks').get(taskId).get('result').put(updates.result);

  res.json({ success: true, task_id: taskId });
});

// DELETE /fleet/tasks/:id - Delete task
app.delete('/fleet/tasks/:id', (req, res) => {
  const taskId = req.params.id;

  gun.get('fleet').get('tasks').get(taskId).put(null);

  res.json({ success: true, deleted: taskId });
});

// Start server
app.listen(port, '0.0.0.0', () => {
  console.log('='.repeat(60));
  console.log('Gun.js Relay Server for K1 Fleet');
  console.log('='.repeat(60));
  console.log(`HTTP API:  http://0.0.0.0:${port}`);
  console.log(`Gun sync:  ws://0.0.0.0:${port}/gun`);
  console.log('');
  console.log('Endpoints:');
  console.log('  GET  /health');
  console.log('  GET  /fleet/robots');
  console.log('  PUT  /fleet/robots/:id');
  console.log('  GET  /fleet/tasks');
  console.log('  GET  /fleet/tasks/:id');
  console.log('  PUT  /fleet/tasks/:id');
  console.log('  PATCH /fleet/tasks/:id');
  console.log('  DELETE /fleet/tasks/:id');
  console.log('='.repeat(60));
});
```

Create `gun-relay/docker-compose.yml`:
```yaml
version: '3.8'

services:
  gun-relay:
    build: .
    ports:
      - "8765:8765"
    volumes:
      - ./data:/app/data
    restart: unless-stopped
    environment:
      - PORT=8765
    healthcheck:
      test: ["CMD", "wget", "--quiet", "--tries=1", "--spider", "http://localhost:8765/health"]
      interval: 30s
      timeout: 10s
      retries: 3
```

**Deploy:**
```bash
cd gun-relay
docker-compose up -d

# View logs
docker-compose logs -f

# Check health
curl http://localhost:8765/health
```

---

### Option B: Direct Node.js (Ubuntu Server)

```bash
# Install Node.js 18
curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
sudo apt-get install -y nodejs

# Create project
mkdir -p ~/gun-relay
cd ~/gun-relay
npm init -y
npm install gun express cors body-parser

# Copy server.js from above

# Run with PM2 (process manager)
sudo npm install -g pm2
pm2 start server.js --name gun-relay
pm2 startup
pm2 save

# Check status
pm2 status
pm2 logs gun-relay
```

---

## 🧪 Testing the Server

### Test with curl:
```bash
# Health check
curl http://localhost:8765/health

# Publish robot heartbeat
curl -X PUT http://localhost:8765/fleet/robots/k1_001 \
  -H "Content-Type: application/json" \
  -d '{
    "robot_id": "k1_001",
    "battery": 95.5,
    "mode": "walking",
    "status": "idle",
    "position": {"x": 0, "y": 0, "z": 0},
    "timestamp": 1234567890
  }'

# Get all robots
curl http://localhost:8765/fleet/robots

# Create a task
curl -X PUT http://localhost:8765/fleet/tasks/task_001 \
  -H "Content-Type: application/json" \
  -d '{
    "task_id": "task_001",
    "description": "Navigate to waypoint A",
    "status": "pending",
    "claimed_by": null
  }'

# Get all tasks
curl http://localhost:8765/fleet/tasks
```

### Test with Python:
```python
import requests
import time

RELAY_URL = "http://localhost:8765"

# Publish heartbeat
heartbeat = {
    "robot_id": "k1_test",
    "battery": 100.0,
    "status": "idle",
    "timestamp": time.time()
}

response = requests.put(
    f"{RELAY_URL}/fleet/robots/k1_test",
    json=heartbeat
)
print(response.json())

# Get all robots
robots = requests.get(f"{RELAY_URL}/fleet/robots").json()
print(robots)
```

---

## 🚀 Integrate with K1 Fleet

### 1. Update fleet_coordinator.py
The code is already configured! Just update the URL:

```bash
# Start coordinator with your Gun.js server
python src/fleet_coordinator.py k1_001 http://YOUR_SERVER_IP:8765
```

### 2. Multi-Robot Deployment

**On Robot 001:**
```bash
# Start ROS2 bridge
python src/booster_ros2_bridge.py k1_001 &

# Start fleet coordinator
python src/fleet_coordinator.py k1_001 http://gun-server:8765 &

# Start vision system
python src/came_yolo.py --tensorrt &
```

**On Robot 002:**
```bash
python src/booster_ros2_bridge.py k1_002 &
python src/fleet_coordinator.py k1_002 http://gun-server:8765 &
python src/came_yolo.py --tensorrt &
```

**On Robot 003:**
```bash
python src/booster_ros2_bridge.py k1_003 &
python src/fleet_coordinator.py k1_003 http://gun-server:8765 &
python src/came_yolo.py --tensorrt &
```

### 3. Create Tasks Programmatically

```python
import requests
import time

RELAY_URL = "http://gun-server:8765"

# Create a navigation task
task = {
    "task_id": f"nav_{int(time.time())}",
    "description": "Navigate to charging station",
    "status": "pending",
    "claimed_by": None,
    "waypoint": {"x": 10, "y": 5, "z": 0},
    "created_at": time.time()
}

requests.put(
    f"{RELAY_URL}/fleet/tasks/{task['task_id']}",
    json=task
)

print(f"Task created: {task['task_id']}")
# Robots will automatically discover and claim this task!
```

---

## 📊 Monitoring Dashboard (Optional)

Create `gun-relay/dashboard.html`:
```html
<!DOCTYPE html>
<html>
<head>
    <title>K1 Fleet Dashboard</title>
    <style>
        body { font-family: monospace; background: #1e1e1e; color: #00ff00; padding: 20px; }
        .robot { border: 1px solid #00ff00; padding: 10px; margin: 10px 0; }
        .task { border: 1px solid #ffff00; padding: 10px; margin: 10px 0; }
        .offline { opacity: 0.5; }
    </style>
</head>
<body>
    <h1>K1 Fleet Status</h1>
    <div id="robots"></div>
    <h2>Tasks</h2>
    <div id="tasks"></div>

    <script>
        const RELAY = 'http://localhost:8765';

        async function updateStatus() {
            // Fetch robots
            const robots = await fetch(`${RELAY}/fleet/robots`).then(r => r.json());
            const robotsDiv = document.getElementById('robots');
            robotsDiv.innerHTML = '';

            for (const [id, data] of Object.entries(robots)) {
                const age = Date.now() - (data.timestamp * 1000);
                const online = age < 5000;

                robotsDiv.innerHTML += `
                    <div class="robot ${online ? '' : 'offline'}">
                        <strong>${id}</strong> - ${data.status} - Battery: ${data.battery}%
                        <br>Mode: ${data.mode} | Task: ${data.current_task || 'none'}
                        <br>Age: ${Math.round(age/1000)}s ${online ? '🟢' : '🔴'}
                    </div>
                `;
            }

            // Fetch tasks
            const tasks = await fetch(`${RELAY}/fleet/tasks`).then(r => r.json());
            const tasksDiv = document.getElementById('tasks');
            tasksDiv.innerHTML = '';

            for (const [id, data] of Object.entries(tasks)) {
                tasksDiv.innerHTML += `
                    <div class="task">
                        <strong>${id}</strong> - ${data.status}
                        <br>Claimed by: ${data.claimed_by || 'none'}
                        <br>Description: ${data.description}
                    </div>
                `;
            }
        }

        // Update every 1 second
        updateStatus();
        setInterval(updateStatus, 1000);
    </script>
</body>
</html>
```

Serve dashboard:
```bash
# Add to server.js
app.use(express.static('public'));

# Create public directory
mkdir public
cp dashboard.html public/index.html

# Access at: http://gun-server:8765/
```

---

## 🔒 Security Considerations

### For Production:

1. **Add Authentication:**
```javascript
// In server.js
const API_KEY = process.env.API_KEY || 'your-secret-key';

app.use((req, res, next) => {
  if (req.path === '/health') return next();

  const key = req.headers['x-api-key'];
  if (key !== API_KEY) {
    return res.status(401).json({ error: 'Unauthorized' });
  }
  next();
});
```

Update Python clients:
```python
headers = {'X-API-Key': 'your-secret-key'}
requests.put(url, json=data, headers=headers)
```

2. **Add HTTPS:**
```bash
# Use nginx reverse proxy
sudo apt install nginx certbot python3-certbot-nginx

# Configure nginx
sudo nano /etc/nginx/sites-available/gun-relay

# Get SSL cert
sudo certbot --nginx -d gun-relay.yourcompany.com
```

3. **Rate Limiting:**
```javascript
const rateLimit = require('express-rate-limit');

const limiter = rateLimit({
  windowMs: 1000, // 1 second
  max: 100 // 100 requests per second
});

app.use(limiter);
```

---

## 🐛 Troubleshooting

### Robots not seeing each other:
```bash
# Check Gun.js server logs
pm2 logs gun-relay

# Test connectivity from robot
curl http://gun-server:8765/health

# Check firewall
sudo ufw allow 8765/tcp
```

### Tasks not being claimed:
```bash
# Check task format
curl http://gun-server:8765/fleet/tasks

# Verify robot heartbeat
curl http://gun-server:8765/fleet/robots

# Check Python logs
python src/fleet_coordinator.py k1_001 http://gun-server:8765
```

### Data persistence issues:
```bash
# Check data directory
ls -la gun-relay/data/

# Backup data
tar -czf gun-backup-$(date +%Y%m%d).tar.gz data/
```

---

## 📈 Scaling

For 10+ robots:
- Use Gun.js peers (distributed servers)
- Add Redis cache layer
- Deploy multiple relay servers behind load balancer

For 100+ robots:
- Migrate to RMW (Robot Middleware Framework)
- Add RabbitMQ for task queue
- Use Kubernetes for orchestration

---

## ✅ Quick Start Checklist

- [ ] Install Node.js 18+ or Docker
- [ ] Create `server.js` and `docker-compose.yml`
- [ ] Start server: `docker-compose up -d`
- [ ] Test health: `curl http://localhost:8765/health`
- [ ] Update robot code with server URL
- [ ] Deploy to first robot
- [ ] Verify heartbeat: `curl http://server:8765/fleet/robots`
- [ ] Deploy to additional robots
- [ ] Monitor dashboard at http://server:8765/

---

**Next:** See PROJECT_STRUCTURE.md for full system architecture
