import math     # Mathe
from typing import Optional
from datetime import datetime
from click import DateTime

class SmoothData:
    def __init__(self, alpha_base, delta:float=0.01, adaptivity:float=.2) -> None:
        self.alpha_base = alpha_base
        self.delta = delta
        self.adaptivity = adaptivity
        self.last_smoothed: Optional[float] = None 
        self.last_time: Optional[DateTime] = None
        self.inactive_since: Optional[DateTime] = None  
        """ CONSTANTS """
        self.DEADZONE: float = 0.05             # TODO: Deadzone anpassen -> Werte herausfinden
        self.DYNAMIC_THRESHOLD: float = 0.1     # Schwelle für "dynamische" Anpassung (bei hoher Beschleunigung)
        self.T_RELAX: float = 0.5               # Zeitkonstante für Rückkehr zu Basis-Alpha
        self.FREQUENCY: float = 30.0

    def smooth_data(self, x_t) -> float:
        """
        Glättet die Daten des Gyrosensor
        Verschiedene Stufen bei starker oder schwacher Bewegung
        1. Starke Bewegung -> Dynamische Anpassung des Alpha-Wertes
        2. Schwache Bewegung -> Zeitbasierte Anpassung des Alpha-Wertes
        3. Innerhalb der Deadzone -> Passthrough
        """
        # Zeitdifferenz
        delta_time: float = (datetime.now() - self.last_time).total_seconds() if self.last_time else 0.0 # type: ignore
        delta_time_target: float = 1.0/self.FREQUENCY      # Im Endeffekt ist das die Periodendauer (also unser Target)
        diff: float = abs(x_t - self.last_smoothed)
        inactive_time: float = (datetime.now() - self.inactive_since).total_seconds() if self.inactive_since else 0.0 # type: ignore

        if self.last_smoothed is None:
            # Erste Messung (muss nicht geglättet werden)
            self.last_smoothed = x_t
            self.last_time = datetime.now() # type: ignore
            self.inactive_since = datetime.now() # type: ignore
            return x_t
        # Alpha berechnen
        if diff > self.DYNAMIC_THRESHOLD:
            """Dynamische Anpassung - bei großen Differenzen"""
            alpha: float = min(1.0, self.alpha_base + self.adaptivity*diff)
        else:
            """Zeitbasierte Abrechnung"""
            alpha: float = 1 - math.exp(-delta_time/delta_time_target)

        smoothed: float = 0.0

        if abs(x_t - self.last_smoothed) < self.DEADZONE:
            # Unter der Deadzone passiert nichts
            smoothed = self.last_smoothed   #type: ignore
            # TODO: T_RELAX einführen
            if self.inactive_since is None:
                self.inactive_since = datetime.now()    #type: ignore 
        else:
            # Standard-Glättung
            smoothed = alpha*x_t+(1-alpha)*self.last_smoothed #if self.last_smoothed is not None else x_t

        # TODO: Multi-Achsen implementieren
        # -> entweder drei Instanzen der Funktion oder Listen als Input/Output

        self.last_time = datetime.now() # type: ignore
        self.last_smoothed = smoothed
        return smoothed
