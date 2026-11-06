#!/usr/bin/env python3
"""Grafica las mediciones guardadas; requiere matplotlib, no inicia Gazebo."""

import csv
import gzip
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parent


def aligned(profile, case):
    """Lee coordenadas ya alineadas en SE(2) y timestamps simulados."""
    with gzip.open(ROOT/'data'/profile/f'01_{case}_aligned.csv.gz', 'rt') as stream:
        rows = csv.reader(stream)
        next(rows)
        return np.array([[float(v) for v in row] for row in rows])


def main():
    """Compara una repetición basal con una calibrada, sin promediar trayectorias."""
    plt.switch_backend('Agg')
    plt.rcParams.update({'font.size': 11, 'axes.spines.top': False,
                         'axes.spines.right': False, 'svg.fonttype': 'none'})
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    for profile, color, label in [('baseline', '#c77b21', 'Inicial'),
                                  ('calibrated', '#236bb2', 'Calibrada')]:
        for index, case in enumerate(['forward', 'left', 'turn_left', 'arc_left']):
            data = aligned(profile, case)
            ax = axes.flat[index]
            if case == 'arc_left':
                ax.plot(data[:, 1], data[:, 2], color=color, label=label+' · Gazebo')
                ax.plot(data[:, 4], data[:, 5], '--', color=color, label=label+' · odom')
                ax.set(xlabel='X local (m)', ylabel='Y local (m)')
                ax.set_aspect('equal', adjustable='box')
            else:
                column = {'forward': 1, 'left': 2, 'turn_left': 3}[case]
                scale = 180./np.pi if case == 'turn_left' else 1.
                times = data[:, 0]-data[0, 0]
                ax.plot(times, scale*data[:, column], color=color, label=label+' · Gazebo')
                ax.plot(times, scale*data[:, column+3], '--', color=color,
                        label=label+' · odom')
                ax.set(xlabel='Tiempo simulado (s)',
                       ylabel='Yaw relativo (°)' if case == 'turn_left' else 'Desplazamiento (m)')
    titles = ['Avance · 0.40 m', 'Lateral · 0.40 m',
              'Giro · 90°', 'Traslación y giro · 1.0 rad']
    for ax, title in zip(axes.flat, titles):
        ax.set_title(title, loc='left', fontweight='bold')
        ax.grid(alpha=.18)
    handles, labels = axes.flat[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='lower center', ncol=4, frameon=False)
    fig.suptitle('Pepper: odometría frente a pose real de Gazebo', fontsize=16)
    fig.tight_layout(rect=(0, .065, 1, .95))
    fig.savefig(ROOT/'comparison.png', dpi=170)
    fig.savefig(ROOT/'comparison.svg')
    plt.close(fig)


if __name__ == '__main__':
    main()
