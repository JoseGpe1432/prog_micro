"""
detection.py
------------
Resultado de la visión: un objeto en el piso con su POSICIÓN y TAMAÑO REAL medidos
(no un "tipo" aprendido). El tamaño se obtiene de la geometría de la cámara.
"""
from dataclasses import dataclass

DEBRIS = "DEBRIS"      # cabe en el aspirador: ir a recogerlo
TOO_BIG = "TOO_BIG"    # demasiado grande para aspirar: rodearlo
CABLE = "CABLE"        # largo y delgado (cable, cordón): rodearlo, enreda los cepillos
IGNORE = "IGNORE"      # demasiado pequeño o enorme (sombra, junta, alfombra): se descarta

AVOID_KINDS = (TOO_BIG, CABLE)


@dataclass
class Detection:
    x: float            # m, adelante del centro del robot
    y: float            # m, izquierda del centro del robot
    length: float       # m, dimensión mayor medida sobre el piso
    width: float        # m, dimensión menor
    kind: str = DEBRIS
