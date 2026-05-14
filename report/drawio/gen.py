# generating a simple upchirp signal
import numpy as np
import matplotlib.pyplot as plt

# ============================================================
# continuing with the previous code, we just need to now add Symbol s.

# generating a simple upchirp signal

# LoRa parameters required such as BW, Spreading Factor (SF), Sampling Rate ect

BW = 125e3
SF = 7
M = 2**SF  # M represents total number chirps and it depends upon SF
Fs = (
    8 * BW
)  # (2*B Shanon Theoerom but signal visualization we increase it to 8 to make 1MS/s)
Tsym = M / BW  # symbol time
N = int(Tsym * Fs)  # total samples in symbol
t = np.linspace(0, Tsym, N, endpoint=False)

# symbols let us take s=0 for instance, you can change it to see how starting
# frequency is affected

s = 0
# now the starting frequency would be:

start_freq = -BW / 2 + BW * s / M

# instantaneous frequency now takes a wrap around at when reaching the maximum BW

wrap_tim = int((Tsym - Tsym * s / M) * 1e6)
print(wrap_tim)


inst_freq = start_freq + (BW / Tsym * (t[0 : int(wrap_tim)]))


inst_freq = np.append(
    inst_freq,
    start_freq - BW + (BW / Tsym * (t[int(wrap_tim) :])),
)


# np.set_printoptions(threshold=sys.maxsize)
# print(inst_freq)

phase = 2 * np.pi * start_freq * t + 2 * np.pi * (-np.cumsum(inst_freq) / Fs)

css_symbol = np.exp(1j * phase)
plt.figure(figsize=(15, 5))


plt.subplot(1, 2, 1)
plt.plot(t * 1e3, np.real(css_symbol))
plt.title(f"CSS Modulated Signal in Time Domain Symbol {s}")
plt.xlabel("Time [ms]")
plt.ylabel("Amplitude")
plt.grid(True)
plt.tight_layout()

plt.subplot(1, 2, 2)
plt.plot(t * 1e3, inst_freq)
plt.title(f"Instantaneous Frequency of CSS Signal (Symbol {s})")
plt.xlabel("Time [ms]")
plt.ylabel("Frequency [Hz]")
plt.grid()
plt.tight_layout()
plt.savefig("timeVsFreq.svg")
