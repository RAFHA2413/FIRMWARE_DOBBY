# DOBBY V5 - Plan de pruebas para la competencia

**Base conservada:** firmware V3, commit `40117ca`, mejor vuelta registrada de 7,43 s (no demuestra promedio superior al profesor).
**No reemplazar el respaldo.** RAM de pista volátil: reinicio, descarga de firmware o corte de energía = volver a aprender.

## Versiones listas
- `v3-respaldo-743`: V3 sin cambios, vuelta conocida de 7,43 s.
- `v5-01-aprendizaje`: grabación de pista, SIN control predictivo. Para diagnosticar sensores/encoders.
- `v5-02-replay-seguro`: aprendizaje + MAPA SEGURO (+30 PWM como máximo) + MAPA RAPIDO (+55 PWM como máximo).
- `v5-03-replay-plus`: mismos botones; MAPA RAPIDO hasta +75 PWM con antelación de 55 cm; probar solo tras el anterior.

## Descarga y compilación (Windows / VS Code)
```powershell
cd "C:\Users\Laura Melissa\FIRMWARE_DOBBY"
git fetch origin
git switch -c v5-02-replay-seguro --track origin/v5-02-replay-seguro
git log -1 --oneline
python -m py_compile .\dobby_panel.py
cube-cmake --build .\build\Debug --clean-first
$LASTEXITCODE
```
Solo programar `build\Debug\DOBBY.elf` si `$LASTEXITCODE` es 0. STM32CubeProgrammer: **Download**, NO `Full chip erase`; mantener backup del sector de parámetros en 0x080FE000.

## Procedimiento operativo
1. Calibrar sensores sobre línea negra en la pista real. Ajustes conocidos: V_Bas=455, V_Gir=545, KP/KI/KD=60/0/5, KG=0.10, TRAMO=3, ACEL_R=400, resto valores de la configuración récord.
2. Seleccionar un **punto físico de salida y orientación exactos** en un lugar recto. No hay marca de meta: una línea continua NO permite identificar automáticamente el final.
3. Pulsar **APRENDER**. Correr UNA vuelta supervisada y pulsar **FIN VUELTA** al pasar nuevamente por el mismo punto.
4. Pulsar **ESTADO**; verificar `MAPA estado=2`, puntos positivos, `validos_pct>=60`, longitud coherente y sin error de memoria. Si `estado=3`, mapa lleno; si `estado=4`, aprendizaje inválido.
5. **Sin apagar ni reprogramar**, colocar el robot en el MISMO punto/orientación y pulsar **MAPA SEGURO**. Cronometrar una vuelta y STOP. Se registrará `MAP_RACE perfil=1 boost_ms=... previa_ms=... veto_ms=...`.
6. Repetir 2-3 veces con el mapa en RAM. Luego probar el botón **MAPA RAPIDO** (`perfil=2`) si no hubo salidas de pista. Medir 3 vueltas de cada modo, misma batería y parámetros, una vuelta por RUN/STOP.
7. Detener inmediatamente si pierde la pista o se vuelve errático. Si el mapa se desincroniza, el incremento se retira al fallar las condiciones de línea/gyro; el algoritmo original sigue controlando la trayectoria.

## Contingencias
| Síntoma | Acción |
|---|---|
| Error de compilación, `$LASTEXITCODE` distinto de 0 | NO programar. Recuperar el mensaje del compilador y usar V3. |
| El panel no abre | `python -m py_compile .\dobby_panel.py`; comprobar instalación de pyserial y COM10. |
| `MAPA estado=4` | Volver a APRENDER; una vuelta completa desde el mismo punto. |
| `MAPA estado=3` | Pista demasiado larga para el buffer (≈82 m a 2 cm); detener y no usar el mapa. |
| `MAPA estado=2` pero perfil no arranca | Comprobar línea válida >60 %, robot detenido y que no se haya reiniciado. |
| `MAP_RACE boost_ms=0` | Mapa demasiado exigente o sin zonas seguras; competir con V3. |
| Corre mal o se sale | STOP de inmediato. V3 conocido tiene menor incertidumbre. |
| Cambias de firmware después de aprender | El mapa se borra; repetir APRENDER. |
| Encoder patina / salida inicial distinta | STOP; la distancia no sincroniza con la pista. |

## Análisis de software previo
10 escenarios sintéticos del cálculo predictivo pasaron, incluyendo límites, curva, error lateral, pérdida de línea y fin de mapa; el formato de los puntos (10 bytes) se validó con GCC host. **No hay compilación ARM, simulación de dinámica ni pruebas físicas de V5 todavía**. La decisión competitiva debe basarse en vueltas limpias cronometradas.

## Si hay que regresar inmediatamente a V3
```powershell
git fetch origin
git switch -c v3-respaldo-743 --track origin/v3-respaldo-743
cube-cmake --build .\build\Debug --clean-first
$LASTEXITCODE
```
Si `v3-respaldo-743` ya existe localmente, en lugar de `git switch -c...` usar `git switch v3-respaldo-743`. Programar el nuevo ELF con Download. No borrar completamente Flash.
