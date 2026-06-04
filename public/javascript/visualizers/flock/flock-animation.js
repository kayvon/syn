(function () {
  'use strict';

  var TEXTURE_SIZE = 128;          // 128×128 = 16,384 boids
  var BOID_COUNT   = TEXTURE_SIZE * TEXTURE_SIZE;
  var FIELD_SIZE   = 64;           // velocity/position field resolution
  var PATH_SAMPLES = 512;          // texels in the 1-D path texture

  // ─── Shader sources ───────────────────────────────────────────────────────

  var FIELD_VERT = [
    '#version 300 es',
    'precision highp float;',
    'layout(location=0) in vec2 a_uv;',
    'uniform sampler2D u_pos;',
    'uniform sampler2D u_vel;',
    'out vec2 v_vel;',
    'out vec2 v_pos;',
    'void main() {',
    '  vec4 p = texture(u_pos, a_uv);',
    '  vec4 v = texture(u_vel, a_uv);',
    '  v_vel = v.rg;',
    '  v_pos = p.rg;',
    '  gl_Position = vec4(p.rg * 2.0 - 1.0, 0.0, 1.0);',
    '  gl_PointSize = 3.0;',
    '}'
  ].join('\n');

  var FIELD_FRAG = [
    '#version 300 es',
    'precision highp float;',
    'in vec2 v_vel;',
    'in vec2 v_pos;',
    'layout(location=0) out vec4 velField;',
    'layout(location=1) out vec4 posField;',
    'void main() {',
    '  velField = vec4(v_vel, 1.0, 0.0);',
    '  posField = vec4(v_pos, 0.0, 0.0);',
    '}'
  ].join('\n');

  var SIM_VERT = [
    '#version 300 es',
    'precision highp float;',
    'layout(location=0) in vec2 a_pos;',
    'out vec2 v_uv;',
    'void main() {',
    '  v_uv = a_pos * 0.5 + 0.5;',
    '  gl_Position = vec4(a_pos, 0.0, 1.0);',
    '}'
  ].join('\n');

  var SIM_FRAG = [
    '#version 300 es',
    'precision highp float;',
    'in vec2 v_uv;',
    'uniform sampler2D u_pos;',
    'uniform sampler2D u_vel;',
    'uniform sampler2D u_velField;',
    'uniform sampler2D u_posField;',
    'uniform float u_sep;',
    'uniform float u_coh;',
    'uniform float u_ali;',
    'uniform float u_speed;',
    'uniform float u_blend;',   // 0 = full flock, 1 = full path
    'uniform float u_time;',
    'uniform sampler2D u_pathTex;',   // 1-D path: 512×1 RGBA16F, .rg = (x,y) in [0,1]
    'layout(location=0) out vec4 outPos;',
    'layout(location=1) out vec4 outVel;',

    'const float MAX_SPEED    = 0.006;',
    'const float MAX_FORCE    = 0.0002;',
    'const float CELL         = 1.0 / 64.0;',
    'const float ORBIT_SPEED  = 0.05;',   // path circuits per second

    'vec2 steer(vec2 desired, vec2 vel) {',
    '  float len = length(desired);',
    '  if (len < 1e-5) return vec2(0.0);',
    '  vec2 d = (desired / len) * MAX_SPEED;',
    '  vec2 s = d - vel;',
    '  float sl = length(s);',
    '  return sl > MAX_FORCE ? s * (MAX_FORCE / sl) : s;',
    '}',

    'void main() {',
    '  vec2  pos    = texture(u_pos, v_uv).rg;',
    '  vec2  vel    = texture(u_vel, v_uv).rg;',
    '  float effMax = MAX_SPEED * u_speed;',
    '  vec2  newVel;',

    // Flock velocity
    '  vec4  vf  = texture(u_velField, pos);',
    '  vec4  pf  = texture(u_posField, pos);',
    '  float cnt = vf.b;',

    '  vec2 ali = cnt > 0.5 ? steer(vf.rg / cnt, vel) * u_ali : vec2(0.0);',
    '  vec2 coh = cnt > 0.5 ? steer((pf.rg / cnt) - pos, vel) * u_coh : vec2(0.0);',

    '  float dR  = texture(u_velField, pos + vec2( CELL, 0.0)).b;',
    '  float dL  = texture(u_velField, pos + vec2(-CELL, 0.0)).b;',
    '  float dU  = texture(u_velField, pos + vec2(0.0,  CELL)).b;',
    '  float dD  = texture(u_velField, pos + vec2(0.0, -CELL)).b;',
    '  vec2  grad = vec2(dR - dL, dU - dD);',
    '  float gm   = length(grad);',
    '  vec2  sep  = gm > 1e-4',
    '    ? steer(-(grad / gm) * MAX_SPEED, vel) * u_sep * (cnt / 8.0)',
    '    : vec2(0.0);',
    '  vec2 velFlock = vel + ali + coh + sep;',

    // Path velocity — each boid gets a unique offset along the path, animated by u_time
    '  float col       = floor(v_uv.x * 128.0);',
    '  float row       = floor(v_uv.y * 128.0);',
    '  float t         = fract((row * 128.0 + col) / 16384.0 - u_time * ORBIT_SPEED);',
    '  vec2  target    = texture(u_pathTex, vec2(t, 0.5)).rg;',
    '  vec2  velCircle = vel + steer(target - pos, vel) * 15.0;',

    '  newVel = mix(velFlock, velCircle, u_blend);',

    '  float spd    = length(newVel);',
    '  if (spd > effMax && spd > 0.0) newVel *= effMax / spd;',
    '  float minSpd = effMax * 0.1;',
    '  if (spd < minSpd && spd > 1e-5) newVel *= minSpd / spd;',

    '  vec2 newPos = fract(pos + newVel);',

    '  outPos = vec4(newPos, 0.0, 0.0);',
    '  outVel = vec4(newVel, length(newVel), 0.0);',
    '}'
  ].join('\n');

  var RENDER_VERT = [
    '#version 300 es',
    'precision highp float;',
    'layout(location=0) in vec2 a_uv;',
    'uniform sampler2D u_pos;',
    'uniform sampler2D u_vel;',
    'out vec2  v_vel;',
    'out float v_spd;',
    'void main() {',
    '  vec4 p = texture(u_pos, a_uv);',
    '  vec4 v = texture(u_vel, a_uv);',
    '  v_vel = v.rg;',
    '  v_spd = v.b;',
    '  gl_Position  = vec4(p.rg * 2.0 - 1.0, 0.0, 1.0);',
    '  gl_PointSize = mix(2.0, 6.0, clamp(v.b / 0.003, 0.0, 1.0));',
    '}'
  ].join('\n');

  var RENDER_FRAG = [
    '#version 300 es',
    'precision highp float;',
    'in vec2  v_vel;',
    'in float v_spd;',
    'out vec4 fragColor;',

    'vec3 hsv2rgb(vec3 c) {',
    '  vec4 K = vec4(1.0, 2.0/3.0, 1.0/3.0, 3.0);',
    '  vec3 p = abs(fract(c.xxx + K.xyz) * 6.0 - K.www);',
    '  return c.z * mix(K.xxx, clamp(p - K.xxx, 0.0, 1.0), c.y);',
    '}',

    'void main() {',
    '  float d = length(gl_PointCoord - 0.5);',
    '  if (d > 0.5) discard;',
    '  float alpha  = pow(1.0 - smoothstep(0.1, 0.5, d), 1.5);',
    '  float hue    = atan(v_vel.y, v_vel.x) / 6.28318 + 0.5;',
    '  float bright = mix(0.4, 1.0, clamp(v_spd / 0.003, 0.0, 1.0));',
    '  fragColor    = vec4(hsv2rgb(vec3(hue, 0.85, bright)) * alpha, alpha);',
    '}'
  ].join('\n');

  // ─── WebGL helpers ────────────────────────────────────────────────────────

  function compileProgram(gl, vertSrc, fragSrc) {
    function compile(type, src) {
      var s = gl.createShader(type);
      gl.shaderSource(s, src);
      gl.compileShader(s);
      if (!gl.getShaderParameter(s, gl.COMPILE_STATUS)) {
        console.error('Shader error:', gl.getShaderInfoLog(s));
        return null;
      }
      return s;
    }
    var vert = compile(gl.VERTEX_SHADER,   vertSrc);
    var frag = compile(gl.FRAGMENT_SHADER, fragSrc);
    if (!vert || !frag) return null;
    var prog = gl.createProgram();
    gl.attachShader(prog, vert);
    gl.attachShader(prog, frag);
    gl.linkProgram(prog);
    gl.deleteShader(vert);
    gl.deleteShader(frag);
    if (!gl.getProgramParameter(prog, gl.LINK_STATUS)) {
      console.error('Link error:', gl.getProgramInfoLog(prog));
      return null;
    }
    return prog;
  }

  function createStateTex(gl, size, data) {
    var tex = gl.createTexture();
    gl.bindTexture(gl.TEXTURE_2D, tex);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
    gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA16F, size, size, 0, gl.RGBA, gl.FLOAT, data || null);
    return tex;
  }

  function createFieldTex(gl, size) {
    var tex = gl.createTexture();
    gl.bindTexture(gl.TEXTURE_2D, tex);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
    // REPEAT so toroidal boids at screen edges sample the wrapped field correctly
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.REPEAT);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.REPEAT);
    gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA16F, size, size, 0, gl.RGBA, gl.FLOAT, null);
    return tex;
  }

  function createFBO(gl, textures) {
    var fbo = gl.createFramebuffer();
    gl.bindFramebuffer(gl.FRAMEBUFFER, fbo);
    textures.forEach(function (tex, i) {
      gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0 + i, gl.TEXTURE_2D, tex, 0);
    });
    gl.drawBuffers(textures.map(function (_, i) { return gl.COLOR_ATTACHMENT0 + i; }));
    var st = gl.checkFramebufferStatus(gl.FRAMEBUFFER);
    if (st !== gl.FRAMEBUFFER_COMPLETE) {
      console.error('FBO incomplete, status:', st.toString(16));
    }
    gl.bindFramebuffer(gl.FRAMEBUFFER, null);
    return fbo;
  }

  // 1-D path texture: N×1 RGBA16F, seeds with a circle until SVG loads
  function createPathTex(gl, N) {
    var data = new Float32Array(N * 4);
    for (var i = 0; i < N; i++) {
      var t = (i / N) * Math.PI * 2;
      data[i * 4]     = 0.5 + 0.3 * Math.cos(t);
      data[i * 4 + 1] = 0.5 + 0.3 * Math.sin(t);
    }
    var tex = gl.createTexture();
    gl.bindTexture(gl.TEXTURE_2D, tex);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.REPEAT);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
    gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA16F, N, 1, 0, gl.RGBA, gl.FLOAT, data);
    return tex;
  }

  // Fetches path.svg, samples N arc-length-uniform points, normalises to [0,1],
  // then uploads to the existing path texture.
  function loadSvgPath(gl, tex, N) {
    fetch('/javascript/visualizers/flock/path.svg')
      .then(function (r) { return r.text(); })
      .then(function (text) {
        var doc      = new DOMParser().parseFromString(text, 'image/svg+xml');
        var pathEl   = doc.querySelector('path');
        if (!pathEl) { console.warn('path.svg has no <path> element'); return; }

        // Temporarily insert into DOM so getTotalLength works cross-browser
        var ns      = 'http://www.w3.org/2000/svg';
        var svgWrap = document.createElementNS(ns, 'svg');
        svgWrap.style.cssText = 'position:absolute;visibility:hidden;width:0;height:0;';
        var tmp = document.createElementNS(ns, 'path');
        tmp.setAttribute('d', pathEl.getAttribute('d'));
        svgWrap.appendChild(tmp);
        document.body.appendChild(svgWrap);

        var total = tmp.getTotalLength();
        var rawX = new Float32Array(N);
        var rawY = new Float32Array(N);
        for (var i = 0; i < N; i++) {
          var pt = tmp.getPointAtLength((i / N) * total);
          rawX[i] = pt.x;
          rawY[i] = pt.y;
        }
        document.body.removeChild(svgWrap);

        // Normalise: fit into [margin, 1-margin] preserving aspect ratio, centred
        var minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
        for (var i = 0; i < N; i++) {
          if (rawX[i] < minX) minX = rawX[i];
          if (rawX[i] > maxX) maxX = rawX[i];
          if (rawY[i] < minY) minY = rawY[i];
          if (rawY[i] > maxY) maxY = rawY[i];
        }
        var margin = 0.15;
        var span   = Math.max(maxX - minX, maxY - minY);
        var scale  = (1.0 - 2 * margin) / span;
        var cx     = (minX + maxX) / 2;
        var cy     = (minY + maxY) / 2;

        var data = new Float32Array(N * 4);
        for (var i = 0; i < N; i++) {
          data[i * 4]     = 0.5 + (rawX[i] - cx) * scale;
          data[i * 4 + 1] = 0.5 + (rawY[i] - cy) * scale;
        }

        gl.bindTexture(gl.TEXTURE_2D, tex);
        gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA16F, N, 1, 0, gl.RGBA, gl.FLOAT, data);
        console.log('path.svg loaded and uploaded to GPU (' + N + ' samples)');
      })
      .catch(function (err) {
        console.warn('path.svg not loaded, using circle fallback:', err);
      });
  }

  // Shared between field pass and render pass — both use layout(location=0) in vec2 a_uv
  function createBoidVAO(gl) {
    var uvs = new Float32Array(BOID_COUNT * 2);
    for (var i = 0; i < BOID_COUNT; i++) {
      uvs[i * 2]     = ((i % TEXTURE_SIZE) + 0.5) / TEXTURE_SIZE;
      uvs[i * 2 + 1] = (Math.floor(i / TEXTURE_SIZE) + 0.5) / TEXTURE_SIZE;
    }
    var vao = gl.createVertexArray();
    gl.bindVertexArray(vao);
    var vbo = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, vbo);
    gl.bufferData(gl.ARRAY_BUFFER, uvs, gl.STATIC_DRAW);
    gl.enableVertexAttribArray(0);
    gl.vertexAttribPointer(0, 2, gl.FLOAT, false, 0, 0);
    gl.bindVertexArray(null);
    return vao;
  }

  // Fullscreen quad for simulation pass — layout(location=0) in vec2 a_pos
  function createQuadVAO(gl) {
    var vao = gl.createVertexArray();
    gl.bindVertexArray(vao);
    var vbo = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, vbo);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([-1,-1, 1,-1, -1,1, 1,1]), gl.STATIC_DRAW);
    gl.enableVertexAttribArray(0);
    gl.vertexAttribPointer(0, 2, gl.FLOAT, false, 0, 0);
    gl.bindVertexArray(null);
    return vao;
  }

  // ─── Main ─────────────────────────────────────────────────────────────────

  document.addEventListener('DOMContentLoaded', function () {

    var canvas = document.createElement('canvas');
    canvas.style.position = 'absolute';
    canvas.style.top      = '0';
    canvas.style.left     = '0';
    document.body.appendChild(canvas);

    function resize() {
      canvas.width  = window.innerWidth;
      canvas.height = window.innerHeight;
    }
    window.addEventListener('resize', resize);
    resize();

    var gl = canvas.getContext('webgl2', { antialias: false, alpha: false, depth: false });
    if (!gl) {
      console.error('WebGL2 not supported');
      return;
    }
    if (!gl.getExtension('EXT_color_buffer_float')) {
      console.error('EXT_color_buffer_float required but not supported');
      return;
    }

    var fieldProg  = compileProgram(gl, FIELD_VERT,  FIELD_FRAG);
    var simProg    = compileProgram(gl, SIM_VERT,    SIM_FRAG);
    var renderProg = compileProgram(gl, RENDER_VERT, RENDER_FRAG);
    if (!fieldProg || !simProg || !renderProg) return;

    function ul(prog, name) { return gl.getUniformLocation(prog, name); }
    var fU = { pos: ul(fieldProg, 'u_pos'), vel: ul(fieldProg, 'u_vel') };
    var sU = {
      pos:      ul(simProg, 'u_pos'),
      vel:      ul(simProg, 'u_vel'),
      velField: ul(simProg, 'u_velField'),
      posField: ul(simProg, 'u_posField'),
      sep:       ul(simProg, 'u_sep'),
      coh:       ul(simProg, 'u_coh'),
      ali:       ul(simProg, 'u_ali'),
      speed:     ul(simProg, 'u_speed'),
      blend:     ul(simProg, 'u_blend'),
      time:      ul(simProg, 'u_time')
    };
    var rU = { pos: ul(renderProg, 'u_pos'), vel: ul(renderProg, 'u_vel') };

    // Sampler bindings are program state — assign texture units once at init
    gl.useProgram(fieldProg);
    gl.uniform1i(fU.pos, 0);  gl.uniform1i(fU.vel, 1);

    gl.useProgram(simProg);
    gl.uniform1i(sU.pos, 0);  gl.uniform1i(sU.vel, 1);
    gl.uniform1i(sU.velField, 2);  gl.uniform1i(sU.posField, 3);
    gl.uniform1i(ul(simProg, 'u_pathTex'), 4);

    gl.useProgram(renderProg);
    gl.uniform1i(rU.pos, 0);  gl.uniform1i(rU.vel, 1);

    // Random initial state
    var posData = new Float32Array(BOID_COUNT * 4);
    var velData = new Float32Array(BOID_COUNT * 4);
    for (var i = 0; i < BOID_COUNT; i++) {
      posData[i * 4]     = Math.random();
      posData[i * 4 + 1] = Math.random();
      var angle = Math.random() * Math.PI * 2;
      var spd   = 0.001 + Math.random() * 0.001;
      velData[i * 4]     = Math.cos(angle) * spd;
      velData[i * 4 + 1] = Math.sin(angle) * spd;
      velData[i * 4 + 2] = spd;
    }

    var posTex = [
      createStateTex(gl, TEXTURE_SIZE, posData),
      createStateTex(gl, TEXTURE_SIZE, null)
    ];
    var velTex = [
      createStateTex(gl, TEXTURE_SIZE, velData),
      createStateTex(gl, TEXTURE_SIZE, null)
    ];
    var velFieldTex = createFieldTex(gl, FIELD_SIZE);
    var posFieldTex = createFieldTex(gl, FIELD_SIZE);

    var simFBO = [
      createFBO(gl, [posTex[0], velTex[0]]),
      createFBO(gl, [posTex[1], velTex[1]])
    ];
    var fieldFBO = createFBO(gl, [velFieldTex, posFieldTex]);

    var pathTex = createPathTex(gl, PATH_SAMPLES);
    loadSvgPath(gl, pathTex, PATH_SAMPLES);

    var boidVAO = createBoidVAO(gl);
    var quadVAO = createQuadVAO(gl);

    var BLEND_DURATION = 3.0;  // seconds for flock ↔ encircle transition

    var blend       = 0.0;
    var blendTarget = 0.0;
    var startTime   = performance.now();
    var lastTime    = startTime;

    canvas.style.cursor = 'pointer';
    canvas.addEventListener('click', function () {
      blendTarget = blendTarget > 0.5 ? 0.0 : 1.0;
    });
    canvas.addEventListener('touchstart', function (e) {
      e.preventDefault();
      blendTarget = blendTarget > 0.5 ? 0.0 : 1.0;
    });

    // ─── Render loop ──────────────────────────────────────────────

    var readIdx = 0;

    function frame(now) {
      requestAnimationFrame(frame);

      var dt = Math.min((now - lastTime) / 1000.0, 0.05);
      lastTime = now;

      // Linearly advance blend toward target over BLEND_DURATION seconds
      var blendStep = dt / BLEND_DURATION;
      if (blendTarget > blend) blend = Math.min(blend + blendStep, blendTarget);
      else                     blend = Math.max(blend - blendStep, blendTarget);

      var writeIdx = 1 - readIdx;

      var fs    = window.FlockState || {};
      var sep   = fs.separationFactor != null ? fs.separationFactor : 0.001;
      var coh   = fs.cohesionFactor   != null ? fs.cohesionFactor   : 0.0005;
      var ali   = fs.alignmentFactor  != null ? fs.alignmentFactor  : 0.008;
      var speed = (fs.speedFactor     != null ? fs.speedFactor      : 1000) / 1000.0;

      // ── Pass 1: Accumulate field ───────────────────────────────
      gl.useProgram(fieldProg);
      gl.bindFramebuffer(gl.FRAMEBUFFER, fieldFBO);
      gl.viewport(0, 0, FIELD_SIZE, FIELD_SIZE);
      gl.clearColor(0, 0, 0, 0);
      gl.clear(gl.COLOR_BUFFER_BIT);

      gl.enable(gl.BLEND);
      gl.blendEquation(gl.FUNC_ADD);
      gl.blendFunc(gl.ONE, gl.ONE);

      gl.activeTexture(gl.TEXTURE0);
      gl.bindTexture(gl.TEXTURE_2D, posTex[readIdx]);
      gl.activeTexture(gl.TEXTURE1);
      gl.bindTexture(gl.TEXTURE_2D, velTex[readIdx]);

      gl.bindVertexArray(boidVAO);
      gl.drawArrays(gl.POINTS, 0, BOID_COUNT);
      gl.bindVertexArray(null);
      gl.disable(gl.BLEND);

      // ── Pass 2: Simulate ───────────────────────────────────────
      gl.useProgram(simProg);
      gl.bindFramebuffer(gl.FRAMEBUFFER, simFBO[writeIdx]);
      gl.viewport(0, 0, TEXTURE_SIZE, TEXTURE_SIZE);

      gl.activeTexture(gl.TEXTURE0);
      gl.bindTexture(gl.TEXTURE_2D, posTex[readIdx]);
      gl.activeTexture(gl.TEXTURE1);
      gl.bindTexture(gl.TEXTURE_2D, velTex[readIdx]);
      gl.activeTexture(gl.TEXTURE2);
      gl.bindTexture(gl.TEXTURE_2D, velFieldTex);
      gl.activeTexture(gl.TEXTURE3);
      gl.bindTexture(gl.TEXTURE_2D, posFieldTex);
      gl.activeTexture(gl.TEXTURE4);
      gl.bindTexture(gl.TEXTURE_2D, pathTex);

      gl.uniform1f(sU.sep,      sep);
      gl.uniform1f(sU.coh,      coh);
      gl.uniform1f(sU.ali,      ali);
      gl.uniform1f(sU.speed,    speed);
      gl.uniform1f(sU.blend, blend);
      gl.uniform1f(sU.time,  (now - startTime) / 1000.0);

      gl.bindVertexArray(quadVAO);
      gl.drawArrays(gl.TRIANGLE_STRIP, 0, 4);
      gl.bindVertexArray(null);

      // ── Pass 3: Render to screen ───────────────────────────────
      gl.useProgram(renderProg);
      gl.bindFramebuffer(gl.FRAMEBUFFER, null);
      gl.viewport(0, 0, canvas.width, canvas.height);
      gl.clearColor(0, 0, 0, 1);
      gl.clear(gl.COLOR_BUFFER_BIT);

      gl.enable(gl.BLEND);
      gl.blendEquation(gl.FUNC_ADD);
      gl.blendFunc(gl.SRC_ALPHA, gl.ONE);

      gl.activeTexture(gl.TEXTURE0);
      gl.bindTexture(gl.TEXTURE_2D, posTex[writeIdx]);
      gl.activeTexture(gl.TEXTURE1);
      gl.bindTexture(gl.TEXTURE_2D, velTex[writeIdx]);

      gl.bindVertexArray(boidVAO);
      gl.drawArrays(gl.POINTS, 0, BOID_COUNT);
      gl.bindVertexArray(null);
      gl.disable(gl.BLEND);

      readIdx = writeIdx;
    }

    requestAnimationFrame(frame);
  });

})();
