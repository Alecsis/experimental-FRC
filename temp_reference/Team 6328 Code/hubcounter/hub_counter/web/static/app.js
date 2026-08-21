/**
 * Hub Counter Dashboard Application
 * Team 6328 - FRC Ball Counter
 */

class HubCounterApp {
    constructor() {
        this.ws = null;
        this.reconnectInterval = null;
        this.previousTotal = 0;
        this.isExternal = false;
        this.pauseCounting = false;
        this.config = {
            thresholds: { yellow: 100, blue: 360 },
            colors: {
                red: [255, 0, 0],
                yellow: [255, 200, 0],
                blue: [0, 100, 255]
            }
        };

        this.init();
    }

    init() {
        this.bindNavigation();
        this.bindDashboard();
        this.bindExternalControl();
        this.bindConfig();
        this.setupConfettiCanvas();
        this.loadConfig();
        this.loadStatus();
        this.connectWebSocket();
    }

    // Confetti System
    setupConfettiCanvas() {
        this.confettiCanvas = document.createElement('canvas');
        this.confettiCanvas.id = 'confetti-canvas';
        document.body.appendChild(this.confettiCanvas);
        this.confettiCtx = this.confettiCanvas.getContext('2d');
        this.confettiParticles = [];
        this.confettiAnimating = false;

        this.resizeConfettiCanvas();
        window.addEventListener('resize', () => this.resizeConfettiCanvas());
    }

    resizeConfettiCanvas() {
        this.confettiCanvas.width = window.innerWidth;
        this.confettiCanvas.height = window.innerHeight;
    }

    triggerMilestone(type) {
        const color = type === 'yellow' ? 'yellow' : 'blue';
        const isBlue = type === 'blue';

        // Screen flash
        const flash = document.createElement('div');
        flash.className = `milestone-flash ${color}`;
        document.body.appendChild(flash);
        setTimeout(() => flash.remove(), 600);

        // Screen shake (more intense for blue)
        document.body.classList.add(isBlue ? 'shake-intense' : 'shake');
        setTimeout(() => {
            document.body.classList.remove('shake', 'shake-intense');
        }, isBlue ? 2000 : 800);

        // Confetti explosion
        this.launchConfetti(isBlue ? 300 : 150, color);
    }

    launchConfetti(count, colorType) {
        const colors = colorType === 'yellow'
            ? ['#FBC30C', '#FFD700', '#FFA500', '#FFEC8B', '#F4F4EE']
            : ['#446CE3', '#2F4E94', '#00BFFF', '#87CEEB', '#F4F4EE'];

        for (let i = 0; i < count; i++) {
            this.confettiParticles.push({
                x: Math.random() * this.confettiCanvas.width,
                y: -20 - Math.random() * 100,
                vx: (Math.random() - 0.5) * 15,
                vy: Math.random() * 3 + 2,
                size: Math.random() * 12 + 6,
                color: colors[Math.floor(Math.random() * colors.length)],
                rotation: Math.random() * 360,
                rotationSpeed: (Math.random() - 0.5) * 15,
                shape: Math.random() > 0.5 ? 'rect' : 'circle',
                gravity: 0.15 + Math.random() * 0.1,
                drag: 0.98 + Math.random() * 0.015,
                wobble: Math.random() * Math.PI * 2,
                wobbleSpeed: 0.05 + Math.random() * 0.1
            });
        }

        if (!this.confettiAnimating) {
            this.confettiAnimating = true;
            this.animateConfetti();
        }
    }

    animateConfetti() {
        this.confettiCtx.clearRect(0, 0, this.confettiCanvas.width, this.confettiCanvas.height);

        this.confettiParticles = this.confettiParticles.filter(p => {
            // Update physics
            p.vy += p.gravity;
            p.vx *= p.drag;
            p.vy *= p.drag;
            p.x += p.vx + Math.sin(p.wobble) * 2;
            p.y += p.vy;
            p.rotation += p.rotationSpeed;
            p.wobble += p.wobbleSpeed;

            // Draw particle
            this.confettiCtx.save();
            this.confettiCtx.translate(p.x, p.y);
            this.confettiCtx.rotate(p.rotation * Math.PI / 180);
            this.confettiCtx.fillStyle = p.color;

            if (p.shape === 'rect') {
                this.confettiCtx.fillRect(-p.size / 2, -p.size / 4, p.size, p.size / 2);
            } else {
                this.confettiCtx.beginPath();
                this.confettiCtx.arc(0, 0, p.size / 2, 0, Math.PI * 2);
                this.confettiCtx.fill();
            }

            this.confettiCtx.restore();

            // Keep particle if still on screen
            return p.y < this.confettiCanvas.height + 50;
        });

        if (this.confettiParticles.length > 0) {
            requestAnimationFrame(() => this.animateConfetti());
        } else {
            this.confettiAnimating = false;
        }
    }

    checkMilestones(oldTotal, newTotal) {
        const yellow = this.config.thresholds.yellow;
        const blue = this.config.thresholds.blue;

        // Check if we crossed blue threshold (360)
        if (oldTotal < blue && newTotal >= blue) {
            this.triggerMilestone('blue');
        }
        // Check if we crossed yellow threshold (100)
        else if (oldTotal < yellow && newTotal >= yellow) {
            this.triggerMilestone('yellow');
        }
    }

    // Navigation
    bindNavigation() {
        document.querySelectorAll('.nav-btn').forEach(btn => {
            btn.addEventListener('click', () => {
                const view = btn.dataset.view;
                this.switchView(view);
            });
        });
    }

    switchView(viewName) {
        // Update nav buttons
        document.querySelectorAll('.nav-btn').forEach(btn => {
            btn.classList.toggle('active', btn.dataset.view === viewName);
        });

        // Update views
        document.querySelectorAll('.view').forEach(view => {
            view.classList.toggle('active', view.id === `${viewName}-view`);
        });

        // Reload config when switching to config view
        if (viewName === 'config') {
            this.loadConfig();
        }
    }

    // Dashboard
    bindDashboard() {
        // Reset button
        document.getElementById('reset-btn').addEventListener('click', () => {
            this.resetCounts();
        });

        // Simulate buttons with hold-to-rapid-fire
        document.querySelectorAll('.simulate-btn').forEach(btn => {
            let holdState = null;

            const startHold = (e) => {
                e.preventDefault();
                const channel = parseInt(btn.dataset.channel);

                // Trigger immediately on press
                this.simulateBall(channel);

                // Start rapid fire after initial press
                holdState = {
                    channel,
                    startTime: Date.now(),
                    intervalId: null
                };

                // Start at 4x per second (250ms)
                const scheduleNext = () => {
                    if (!holdState) return;

                    // Calculate current rate based on hold duration
                    // Ramp from 4/sec to 20/sec over 4 seconds
                    const elapsed = (Date.now() - holdState.startTime) / 1000;
                    const rate = Math.min(4 + (elapsed * 4), 20); // 4 to 20 over ~4 seconds
                    const interval = 1000 / rate;

                    holdState.intervalId = setTimeout(() => {
                        if (holdState) {
                            this.simulateBall(holdState.channel);
                            scheduleNext();
                        }
                    }, interval);
                };

                scheduleNext();
            };

            const stopHold = () => {
                if (holdState) {
                    if (holdState.intervalId) {
                        clearTimeout(holdState.intervalId);
                    }
                    holdState = null;
                }
            };

            // Mouse events
            btn.addEventListener('mousedown', startHold);
            btn.addEventListener('mouseup', stopHold);
            btn.addEventListener('mouseleave', stopHold);

            // Touch events for mobile
            btn.addEventListener('touchstart', startHold);
            btn.addEventListener('touchend', stopHold);
            btn.addEventListener('touchcancel', stopHold);
        });
    }

    // External Control
    bindExternalControl() {
        document.getElementById('release-control-btn').addEventListener('click', () => {
            this.dismissExternal();
        });
    }

    async dismissExternal() {
        try {
            const response = await fetch('/api/external/dismiss', { method: 'POST' });
            const data = await response.json();
            if (data.success) {
                this.showToast('External control released', 'success');
            }
        } catch (error) {
            console.error('Dismiss external error:', error);
            this.showToast('Failed to release control', 'error');
        }
    }

    updateExternalState(data) {
        this.isExternal = data.is_external;
        if (data.pause_counting !== undefined) {
            this.pauseCounting = data.pause_counting;
        }
        const banner = document.getElementById('external-banner');
        if (data.is_external) {
            document.getElementById('external-pattern').textContent = data.pattern || '-';
            document.getElementById('external-color-text').textContent = data.color || '-';
            const swatch = document.getElementById('external-color-swatch');
            if (data.color) {
                swatch.style.backgroundColor = data.color;
                swatch.style.display = 'inline-block';
            } else {
                swatch.style.display = 'none';
            }
            banner.style.display = 'flex';
        } else {
            banner.style.display = 'none';
        }
        this.updatePausedDisplay();
    }

    updatePausedDisplay() {
        const pausedDisplay = document.getElementById('paused-count-display');
        if (this.isExternal && this.pauseCounting) {
            pausedDisplay.style.display = 'block';
        } else {
            pausedDisplay.style.display = 'none';
        }
    }

    updateCounts(counts) {
        const { channels, total, paused_total } = counts;

        // Update paused count
        if (paused_total !== undefined) {
            document.getElementById('paused-count').textContent = paused_total;
        }

        // Update channel counts
        channels.forEach((count, i) => {
            const el = document.getElementById(`channel-${i}`);
            if (el && el.textContent !== String(count)) {
                el.textContent = count;
                this.pulseElement(el.closest('.channel-card'));
            }
        });

        // Update total
        const totalEl = document.getElementById('total-count');
        const oldTotal = this.previousTotal;

        if (totalEl.textContent !== String(total)) {
            totalEl.textContent = total;
            this.pulseElement(totalEl);

            // Check for milestone crossings
            this.checkMilestones(oldTotal, total);
        }

        // Track previous total for milestone detection
        this.previousTotal = total;

        // Update color based on thresholds
        this.updateTotalColor(total);
    }

    updateTotalColor(total) {
        const totalEl = document.getElementById('total-count');
        totalEl.classList.remove('color-red', 'color-yellow', 'color-blue');

        if (total >= this.config.thresholds.blue) {
            totalEl.classList.add('color-blue');
        } else if (total >= this.config.thresholds.yellow) {
            totalEl.classList.add('color-yellow');
        } else {
            totalEl.classList.add('color-red');
        }
    }

    pulseElement(el) {
        el.classList.add('pulse');
        setTimeout(() => el.classList.remove('pulse'), 500);
    }

    async resetCounts() {
        try {
            const response = await fetch('/api/counts/reset', { method: 'POST' });
            const data = await response.json();
            if (data.success) {
                this.updateCounts(data.counts);
                this.showToast('Counts reset', 'success');
            }
        } catch (error) {
            console.error('Reset error:', error);
            this.showToast('Failed to reset counts', 'error');
        }
    }

    async simulateBall(channel) {
        try {
            const response = await fetch(`/api/test/simulate-ball/${channel}`, {
                method: 'POST'
            });
            const data = await response.json();
            if (data.success) {
                this.updateCounts(data.counts);
            }
        } catch (error) {
            console.error('Simulate error:', error);
        }
    }

    // Config
    bindConfig() {
        document.getElementById('config-form').addEventListener('submit', (e) => {
            e.preventDefault();
            this.saveConfig();
        });

        // Color preview updates
        ['red', 'yellow', 'blue'].forEach(color => {
            ['r', 'g', 'b'].forEach(component => {
                const input = document.getElementById(`color-${color}-${component}`);
                input.addEventListener('input', () => this.updateColorPreview(color));
            });
        });
    }

    async loadConfig() {
        try {
            const response = await fetch('/api/config');
            const config = await response.json();
            this.updateConfig(config);
        } catch (error) {
            console.error('Load config error:', error);
        }
    }

    updateConfig(config) {
        // Store config
        this.config.thresholds = config.thresholds || this.config.thresholds;
        this.config.colors = config.colors || this.config.colors;

        // Update form values
        document.getElementById('team-number').value = config.team_number || 6328;
        document.getElementById('robot-address').value = config.robot_address || '10.63.28.1';
        document.getElementById('threshold-yellow').value = config.thresholds?.yellow || 100;
        document.getElementById('threshold-blue').value = config.thresholds?.blue || 360;

        if (config.colors) {
            ['red', 'yellow', 'blue'].forEach(color => {
                const rgb = config.colors[color];
                if (rgb) {
                    document.getElementById(`color-${color}-r`).value = rgb[0];
                    document.getElementById(`color-${color}-g`).value = rgb[1];
                    document.getElementById(`color-${color}-b`).value = rgb[2];
                    this.updateColorPreview(color);
                }
            });
        }

        // Update current display that depends on config
        this.updateTotalColor(this.previousTotal);
    }

    async saveConfig() {
        const config = {
            team_number: parseInt(document.getElementById('team-number').value),
            robot_address: document.getElementById('robot-address').value,
            thresholds: {
                yellow: parseInt(document.getElementById('threshold-yellow').value),
                blue: parseInt(document.getElementById('threshold-blue').value)
            },
            colors: {
                red: [
                    parseInt(document.getElementById('color-red-r').value),
                    parseInt(document.getElementById('color-red-g').value),
                    parseInt(document.getElementById('color-red-b').value)
                ],
                yellow: [
                    parseInt(document.getElementById('color-yellow-r').value),
                    parseInt(document.getElementById('color-yellow-g').value),
                    parseInt(document.getElementById('color-yellow-b').value)
                ],
                blue: [
                    parseInt(document.getElementById('color-blue-r').value),
                    parseInt(document.getElementById('color-blue-g').value),
                    parseInt(document.getElementById('color-blue-b').value)
                ]
            }
        };

        try {
            const response = await fetch('/api/config', {
                method: 'PATCH',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(config)
            });
            const data = await response.json();

            if (data.success) {
                this.config.thresholds = config.thresholds;
                this.config.colors = config.colors;
                this.showToast('Configuration saved', 'success');

                // Update current display
                const totalEl = document.getElementById('total-count');
                this.updateTotalColor(parseInt(totalEl.textContent));
            } else {
                this.showToast('Failed to save configuration', 'error');
            }
        } catch (error) {
            console.error('Save config error:', error);
            this.showToast('Failed to save configuration', 'error');
        }
    }

    updateColorPreview(color) {
        const r = document.getElementById(`color-${color}-r`).value;
        const g = document.getElementById(`color-${color}-g`).value;
        const b = document.getElementById(`color-${color}-b`).value;
        const preview = document.getElementById(`preview-${color}`);
        preview.style.backgroundColor = `rgb(${r}, ${g}, ${b})`;
    }

    // Status
    async loadStatus() {
        try {
            const response = await fetch('/api/status');
            const status = await response.json();

            this.updateCounts(status.counts);
            this.updateNTStatus(status.nt_connected);
            this.updateExternalState({
                is_external: status.is_external || false,
                pattern: '',
                color: '',
            });
        } catch (error) {
            console.error('Load status error:', error);
        }
    }

    updateNTStatus(connected) {
        const dot = document.getElementById('nt-status');
        const text = document.getElementById('nt-status-text');

        dot.classList.toggle('connected', connected);
        text.textContent = connected ? 'Connected' : 'Disconnected';
    }

    updateWSStatus(connected) {
        const dot = document.getElementById('ws-status');
        const text = document.getElementById('ws-status-text');

        dot.classList.toggle('connected', connected);
        text.textContent = connected ? 'Connected' : 'Disconnected';
    }

    // WebSocket
    connectWebSocket() {
        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const wsUrl = `${protocol}//${window.location.host}/api/ws`;

        this.ws = new WebSocket(wsUrl);

        this.ws.onopen = () => {
            console.log('WebSocket connected');
            this.updateWSStatus(true);

            // Clear reconnect interval
            if (this.reconnectInterval) {
                clearInterval(this.reconnectInterval);
                this.reconnectInterval = null;
            }

            // Start keepalive
            this.startKeepalive();
        };

        this.ws.onmessage = (event) => {
            try {
                const message = JSON.parse(event.data);
                this.handleWSMessage(message);
            } catch (error) {
                // Might be a pong response
                if (event.data !== 'pong') {
                    console.error('WS message parse error:', error);
                }
            }
        };

        this.ws.onclose = () => {
            console.log('WebSocket disconnected');
            this.updateWSStatus(false);
            this.scheduleReconnect();
        };

        this.ws.onerror = (error) => {
            console.error('WebSocket error:', error);
        };
    }

    handleWSMessage(message) {
        switch (message.type) {
            case 'counts':
                this.updateCounts(message.data);
                break;
            case 'status':
                if (message.data.nt_connected !== undefined) {
                    this.updateNTStatus(message.data.nt_connected);
                }
                break;
            case 'external_state':
                this.updateExternalState(message.data);
                break;
            case 'config':
                this.updateConfig(message.data);
                this.showToast('Configuration updated remotely', 'info');
                break;
            default:
                console.log('Unknown WS message type:', message.type);
        }
    }

    startKeepalive() {
        setInterval(() => {
            if (this.ws && this.ws.readyState === WebSocket.OPEN) {
                this.ws.send('ping');
            }
        }, 30000);
    }

    scheduleReconnect() {
        if (!this.reconnectInterval) {
            this.reconnectInterval = setInterval(() => {
                console.log('Attempting WebSocket reconnect...');
                this.connectWebSocket();
            }, 5000);
        }
    }

    // Toast notifications
    showToast(message, type = 'info') {
        // Remove existing toast
        const existing = document.querySelector('.toast');
        if (existing) {
            existing.remove();
        }

        const toast = document.createElement('div');
        toast.className = `toast ${type}`;
        toast.textContent = message;
        document.body.appendChild(toast);

        // Show
        requestAnimationFrame(() => {
            toast.classList.add('show');
        });

        // Hide after 3 seconds
        setTimeout(() => {
            toast.classList.remove('show');
            setTimeout(() => toast.remove(), 300);
        }, 3000);
    }
}

// Initialize app when DOM is ready
document.addEventListener('DOMContentLoaded', () => {
    window.app = new HubCounterApp();
});
