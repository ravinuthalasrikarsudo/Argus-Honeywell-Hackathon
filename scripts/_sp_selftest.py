#!/usr/bin/env python3
"""SuperPoint ONNX self-test: provider, model I/O signature, inference Hz."""
import time
import numpy as np
import onnxruntime as ort

MODEL = '/home/vittal/argus/models/superpoint/superpoint_1024.onnx'
print('onnxruntime', ort.__version__)
print('available providers:', ort.get_available_providers())
if hasattr(ort, 'preload_dlls'):
    try:
        ort.preload_dlls()
        print('preload_dlls() ok')
    except Exception as e:
        print('preload_dlls() failed:', e)

so = ort.SessionOptions()
so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
sess = ort.InferenceSession(MODEL, sess_options=so,
                            providers=['CUDAExecutionProvider', 'CPUExecutionProvider'])
print('ACTIVE providers:', sess.get_providers())
for i in sess.get_inputs():
    print('  IN ', i.name, i.shape, i.type)
for o in sess.get_outputs():
    print('  OUT', o.name, o.shape, o.type)

in_name = sess.get_inputs()[0].name
for (H, W) in [(720, 1280)]:
    x = (np.random.rand(1, 1, H, W).astype(np.float32))
    outs = sess.run(None, {in_name: x})
    print(f'\n[{H}x{W}] output shapes:', [np.asarray(o).shape for o in outs])
    # warmup
    for _ in range(3):
        sess.run(None, {in_name: x})
    N = 30
    t0 = time.monotonic()
    for _ in range(N):
        sess.run(None, {in_name: x})
    dt = (time.monotonic() - t0) / N
    print(f'[{H}x{W}] mean {dt*1e3:.1f} ms  ->  {1.0/dt:.1f} Hz  ({sess.get_providers()[0]})')
