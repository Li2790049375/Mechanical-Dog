static const char PROGMEM INDEX_HTML[] = R"rawliteral(
<!doctype html>
<html>
<head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width,initial-scale=1">
    <title>ESP32 Mech Dog</title>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/gsap/3.12.5/gsap.min.js"></script>
    <style>
    * { margin: 0; padding: 0; box-sizing: border-box; }

    body {
        font-family: 'Segoe UI', Arial, sans-serif;
        background: linear-gradient(135deg, #0a0a1a 0%, #1a1a2e 50%, #0a0a1a 100%);
        color: #e0e0e0;
        min-height: 100vh;
        overflow-x: hidden;
    }

    h1 {
        text-align: center;
        font-size: 24px;
        padding: 10px 0 0;
        background: linear-gradient(90deg, #00d4ff, #7b2ff7);
        -webkit-background-clip: text;
        -webkit-text-fill-color: transparent;
        letter-spacing: 2px;
    }

    section.main {
        display: flex;
        flex-direction: column;
        align-items: center;
        padding: 10px;
    }

    figure {
        padding: 0;
        margin: 0;
    }

    figure img {
        display: block;
        width: 100%;
        height: auto;
        border-radius: 4px;
        margin-top: 8px;
    }

    @media (min-width:800px) and (orientation:landscape) {
        figure img {
            display: block;
            max-width: 100%;
            max-height: calc(100vh - 40px);
            width: auto;
            height: auto;
        }
    }

    #stream {
        width: 100%;
        height: 100%;
        object-fit: contain;
        background: #1a1a2e;
    }

    section #buttons {
        width: 100%;
        max-width: 640px;
        text-align: center;
        margin: 0 auto;
    }

    .btn {
        display: inline-flex;
        align-items: center;
        justify-content: center;
        padding: 10px 18px;
        border: none;
        border-radius: 10px;
        font-size: 14px;
        font-weight: 600;
        cursor: pointer;
        color: #fff;
        letter-spacing: 0.5px;
        user-select: none;
        -webkit-user-select: none;
        position: relative;
        overflow: hidden;
        transition: box-shadow 0.2s;
        -webkit-touch-callout: none;
    }

    .btn::after {
        content: '';
        position: absolute;
        top: 50%; left: 50%;
        width: 0; height: 0;
        background: rgba(255,255,255,0.2);
        border-radius: 50%;
        transform: translate(-50%, -50%);
        transition: width 0.4s, height 0.4s;
    }

    .btn:active::after {
        width: 200px;
        height: 200px;
    }

    .btn-stream { background: linear-gradient(135deg, #22c55e, #16a34a); }
    .btn-stream:hover { box-shadow: 0 0 20px rgba(34,197,94,0.4); }
    .btn-stream.stop { background: linear-gradient(135deg, #ef4444, #dc2626); }
    .btn-stream.stop:hover { box-shadow: 0 0 20px rgba(239,68,68,0.4); }

    .btn-move { background: linear-gradient(135deg, #00b4d8, #0077b6); }
    .btn-move:hover { box-shadow: 0 0 20px rgba(0,180,216,0.4); }

    .btn-func { background: linear-gradient(135deg, #8b5cf6, #6d28d9); }
    .btn-func:hover { box-shadow: 0 0 20px rgba(139,92,246,0.4); }

    .btn-pwm { background: linear-gradient(135deg, #f97316, #ea580c); font-size: 12px; padding: 8px 12px; }
    .btn-pwm:hover { box-shadow: 0 0 20px rgba(249,115,22,0.4); }

    .image-container {
        position: relative;
        width: 100%;
        max-width: 640px;
        margin: 0 auto;
        min-height: 120px;
    }

    .camera-frame {
        position: relative;
        border-radius: 12px;
        overflow: hidden;
        border: 2px solid rgba(0,212,255,0.3);
        background: #1a1a2e;
        aspect-ratio: 4 / 3;
        min-height: 300px;
        display: flex;
        align-items: center;
        justify-content: center;
    }

    .camera-frame::before {
        content: '';
        position: absolute;
        top: -2px; left: -2px; right: -2px; bottom: -2px;
        border-radius: 14px;
        background: linear-gradient(45deg, #00d4ff, #7b2ff7, #00d4ff, #7b2ff7);
        background-size: 300% 300%;
        z-index: -1;
        animation: borderGlow 3s ease infinite;
    }

    @keyframes borderGlow {
        0%, 100% { background-position: 0% 50%; }
        50% { background-position: 100% 50%; }
    }

    .control-container {
        position: relative;
        text-align: center;
        display: flex;
        flex-direction: column;
        align-items: center;
        padding: 10px 0;
    }

    .camera-controls {
        margin-bottom: 16px;
    }

    .dir-table {
        margin: 0 auto 12px;
        border-spacing: 6px;
    }

    .dir-table .btn { width: 100px; }

    .camera-placeholder {
        width: 100%;
        height: 100%;
        display: flex;
        align-items: center;
        justify-content: center;
        background: #1a1a2e;
        color: #ef4444;
        font-size: 18px;
        font-weight: 600;
        letter-spacing: 1px;
    }

    .camera-offline {
        width: 100%;
        height: 100%;
        display: flex;
        align-items: center;
        justify-content: center;
        background: #1a1a2e;
        color: #00d4ff;
        font-size: 18px;
        font-weight: 600;
        letter-spacing: 1px;
    }

    .btn-pwm-toggle {
        background: linear-gradient(135deg, #64748b, #475569);
        margin: 10px 0;
        font-size: 13px;
        padding: 8px 20px;
    }
    .btn-pwm-toggle:hover { box-shadow: 0 0 20px rgba(100,116,139,0.4); }
    .btn-pwm-toggle.active { background: linear-gradient(135deg, #f97316, #ea580c); }

    @media (max-width: 600px) {
        .dir-table .btn { width: 80px; padding: 8px 10px; font-size: 12px; }
    }

    /* Particle background */
    .particles {
        position: fixed;
        top: 0; left: 0;
        width: 100%; height: 100%;
        pointer-events: none;
        z-index: -1;
        overflow: hidden;
    }

    .particle {
        position: absolute;
        width: 2px; height: 2px;
        background: rgba(0,212,255,0.3);
        border-radius: 50%;
    }

    .hidden { display: none !important; }
    </style>
</head>
<body>
    <div class="particles" id="particles"></div>

    <section class="main">
        <div id="stream-container" class="image-container">
            <div class="camera-frame">
                <div id="camera-placeholder" class="camera-placeholder">STOP</div>
                <div id="camera-offline" class="camera-offline hidden">Camera Offline</div>
                <img id="stream" src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7" class="hidden">
            </div>
        </div>
        <section id="buttons">
            <div id="controls" class="control-container">
                <div class="camera-controls">
                    <button class="btn btn-stream" id="toggle-stream">Start Stream</button>
                </div>
                <table class="dir-table">
                    <tr>
                        <td></td>
                        <td align="center"><button class="btn btn-move" data-cmd="fwd">&#9650;</button></td>
                        <td></td>
                    </tr>
                    <tr>
                        <td align="center"><button class="btn btn-move" data-cmd="left">&#9664;</button></td>
                        <td align="center"><button class="btn btn-move" data-cmd="balance">Balance</button></td>
                        <td align="center"><button class="btn btn-move" data-cmd="right">&#9654;</button></td>
                    </tr>
                    <tr>
                        <td></td>
                        <td align="center"><button class="btn btn-move" data-cmd="back">&#9660;</button></td>
                        <td></td>
                    </tr>
                </table>
                <table class="dir-table">
                    <tr>
                        <td align="center"><button class="btn btn-func" data-cmd="staylow">StayLow</button></td>
                        <td align="center"><button class="btn btn-func" data-cmd="sit">GetDown</button></td>
                        <td align="center"><button class="btn btn-func" data-cmd="jump">Jump</button></td>
                    </tr>
                    <tr>
                        <td align="center"><button class="btn btn-func" data-cmd="nod">Nod</button></td>
                        <td></td>
                        <td align="center"><button class="btn btn-func" data-cmd="spin">Spin</button></td>
                    </tr>
                    <tr>
                        <td align="center"><button class="btn btn-func" data-cmd="initpos">InitPos</button></td>
                        <td></td>
                        <td align="center"><button class="btn btn-func" data-cmd="midpos">MidPos</button></td>
                    </tr>
                    <tr>
                        <td align="center"><button class="btn btn-func" data-cmd="crouch">Crouch</button></td>
                        <td align="center"><button class="btn btn-func" data-cmd="hello">Hello</button></td>
                        <td align="center"><button class="btn btn-func" data-cmd="handshake">HandShake</button></td>
                    </tr>
                </table>
                <button class="btn btn-pwm-toggle" id="pwm-toggle">PWM Adjust</button>
                <table class="dir-table hidden" id="pwm-controls"></table>
            </div>
        </section>
    </section>

<script>
(function() {
    var baseHost = document.location.origin;
    var streamUrl = baseHost + ':81';

    var cmdMap = {
        fwd:       '/control?var=move&val=1&cmd=0',
        left:      '/control?var=move&val=2&cmd=0',
        balance:   '/control?var=funcMode&val=1&cmd=0',
        right:     '/control?var=move&val=4&cmd=0',
        crouch:    '/control?var=funcMode&val=10&cmd=0',
        back:      '/control?var=move&val=5&cmd=0',
        sit:       '/control?var=funcMode&val=11&cmd=0',
        stopMove:  '/control?var=move&val=6&cmd=0',
        staylow:   '/control?var=funcMode&val=2&cmd=0',
        handshake: '/control?var=funcMode&val=3&cmd=0',
        jump:      '/control?var=funcMode&val=4&cmd=0',
        nod:       '/control?var=funcMode&val=5&cmd=0',
        hello:     '/control?var=funcMode&val=6&cmd=0',
        spin:      '/control?var=funcMode&val=7&cmd=0',
        initpos:   '/control?var=funcMode&val=8&cmd=0',
        midpos:    '/control?var=funcMode&val=9&cmd=0'
    };

    // Generate PWM Controls
    var pwmTable = document.getElementById('pwm-controls');
    for (var i = 0; i < 16; i++) {
        var row = document.createElement('tr');
        row.innerHTML =
            '<td align="center"><button class="btn btn-pwm" onclick="fetch(\'' + baseHost + '/control?var=sconfig&val=' + i + '&cmd=-1\')">PWM' + i + '-</button></td>' +
            '<td align="center"><button class="btn btn-pwm" onclick="fetch(\'' + baseHost + '/control?var=sconfig&val=' + i + '&cmd=1\')">PWM' + i + '+</button></td>' +
            '<td align="center"><button class="btn btn-pwm" style="background:linear-gradient(135deg,#64748b,#475569)" onclick="fetch(\'' + baseHost + '/control?var=sset&val=' + i + '&cmd=1\')">SET</button></td>';
        pwmTable.appendChild(row);
    }

    // PWM Toggle
    var pwmToggle = document.getElementById('pwm-toggle');
    pwmToggle.addEventListener('click', function() {
        pwmTable.classList.toggle('hidden');
        pwmToggle.classList.toggle('active');
    });

    // Button Command Binding
    var holdCmds = ['fwd', 'left', 'right', 'back'];
    document.querySelectorAll('[data-cmd]').forEach(function(btn) {
        var cmd = btn.getAttribute('data-cmd');
        if (holdCmds.indexOf(cmd) !== -1) {
            btn.addEventListener('mousedown', function() { fetch(baseHost + cmdMap[cmd]); });
            btn.addEventListener('touchstart', function(e) { e.preventDefault(); fetch(baseHost + cmdMap[cmd]); });
            btn.addEventListener('mouseup', function() { fetch(baseHost + cmdMap.stopMove); });
            btn.addEventListener('mouseleave', function() { fetch(baseHost + cmdMap.stopMove); });
            btn.addEventListener('touchend', function() { fetch(baseHost + cmdMap.stopMove); });
        } else {
            btn.addEventListener('click', function() { fetch(baseHost + cmdMap[cmd]); });
        }
    });

    // Stream Control
    var view = document.getElementById('stream');
    var streamBtn = document.getElementById('toggle-stream');
    var streamContainer = document.getElementById('stream-container');
    var cameraPlaceholder = document.getElementById('camera-placeholder');
    var cameraOffline = document.getElementById('camera-offline');

    function startStream() {
        cameraPlaceholder.classList.add('hidden');
        cameraOffline.classList.add('hidden');
        view.classList.remove('hidden');
        view.src = streamUrl + '/stream';
        view.onerror = function() {
            view.classList.add('hidden');
            cameraOffline.classList.remove('hidden');
        };
        streamBtn.textContent = 'Stop Stream';
        streamBtn.classList.add('stop');
        gsap.fromTo(streamContainer, { scale: 0.95, opacity: 0.5 }, { scale: 1, opacity: 1, duration: 0.4, ease: 'back.out(1.5)' });
    }

    function stopStream() {
        window.stop();
        view.src = '';
        view.onerror = null;
        view.classList.add('hidden');
        cameraOffline.classList.add('hidden');
        cameraPlaceholder.classList.remove('hidden');
        streamBtn.textContent = 'Start Stream';
        streamBtn.classList.remove('stop');
    }

    streamBtn.addEventListener('click', function() {
        if (streamBtn.textContent === 'Stop Stream') {
            stopStream();
        } else {
            startStream();
        }
    });

    // GSAP Animations
    gsap.from('.control-container', { y: 40, opacity: 0, duration: 0.6, ease: 'power3.out', delay: 0.2 });

    // Button press animations (touch + mouse)
    document.querySelectorAll('.btn').forEach(function(btn) {
        btn.addEventListener('mousedown', function() {
            gsap.to(btn, { scale: 0.85, duration: 0.1 });
        });
        btn.addEventListener('mouseup', function() {
            gsap.to(btn, { scale: 1, duration: 0.3, ease: 'back.out(3)' });
        });
        btn.addEventListener('mouseleave', function() {
            gsap.to(btn, { scale: 1, duration: 0.2 });
        });
        btn.addEventListener('touchstart', function() {
            gsap.to(btn, { scale: 0.85, duration: 0.1 });
        }, { passive: true });
        btn.addEventListener('touchend', function() {
            gsap.to(btn, { scale: 1, duration: 0.3, ease: 'back.out(3)' });
        }, { passive: true });
    });

    // Floating Particles
    var particlesEl = document.getElementById('particles');
    for (var p = 0; p < 30; p++) {
        var dot = document.createElement('div');
        dot.className = 'particle';
        dot.style.left = Math.random() * 100 + '%';
        dot.style.top = Math.random() * 100 + '%';
        particlesEl.appendChild(dot);
        gsap.to(dot, {
            y: -80 + Math.random() * 160,
            x: -40 + Math.random() * 80,
            opacity: 0,
            duration: 3 + Math.random() * 4,
            repeat: -1,
            yoyo: true,
            delay: Math.random() * 3,
            ease: 'sine.inOut'
        });
    }

    // Camera frame pulse when streaming
    var cameraFrame = document.querySelector('.camera-frame');
    var pulseAnim = gsap.to(cameraFrame, {
        boxShadow: '0 0 30px rgba(0,212,255,0.5)',
        duration: 1.5,
        repeat: -1,
        yoyo: true,
        ease: 'sine.inOut',
        paused: true
    });

    streamBtn.addEventListener('click', function() {
        if (streamBtn.textContent === 'Stop Stream') {
            pulseAnim.play();
        } else {
            pulseAnim.pause();
            gsap.to(cameraFrame, { boxShadow: '0 0 0px rgba(0,212,255,0)', duration: 0.5 });
        }
    });
})();
</script>
</body>
</html>
)rawliteral";
