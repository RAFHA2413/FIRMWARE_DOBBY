# DOBBY V5.5 — Mapa persistente en Flash interna

## Qué cambia

La V5.2 solo conservaba la pista en RAM. Esta versión guarda automáticamente el mapa en la Flash del STM32H562 al pulsar **FIN VUELTA**, después de detener los motores, y vuelve a cargarlo al encender. Los perfiles **MAPA SEGURO** y **MAPA RÁPIDO** permanecen con las mismas consignas que V5.2.

## Regiones protegidas en el firmware

- `FLASH_MAPA_DIR = 0x080DE000` (128 KiB): datos del mapa.
- `FLASH_PARAM_DIR = 0x080FE000` (8 KiB): parámetros PID y velocidades (NO se borran al guardar mapa).
- La rutina existente `Flash_guardar()` revisa la dirección y si el programa invade la zona reservada. Flash erase en sectores de 8 KiB y escritura de 16 bytes.
- Un mapa tiene hasta 4.096 puntos de 10 bytes (=40 KiB), con cabecera de 36 bytes. Cabecera y puntos tienen CRC32 separados.

## Descarga en VS Code Windows

```powershell
cd "C:\Users\Laura Melissa\FIRMWARE_DOBBY"
git fetch origin
git switch -c v5-05-mapa-flash --track origin/v5-05-mapa-flash
git log -1 --oneline
python -m py_compile .\dobby_panel.py
cube-cmake --build .\build\Debug --clean-first
$LASTEXITCODE
```

Si la rama ya existe localmente: `git switch v5-05-mapa-flash` y `git pull --ff-only`.
**Solo programar si `$LASTEXITCODE` es 0.** Los cambios aún no se han compilado en el entorno ARM real.

En STM32CubeProgrammer descargar `build\Debug\DOBBY.elf` con **Download** y sin usar **Full chip erase**. En primera actualización se pierde el mapa RAM anterior; se debe APRENDER una vez.

## Procedimiento de aceptación, con motores supervisados

1. Tras flashear, iniciar DOBBY. Antes de guardar por primera vez puede anunciar `MAPA FLASH: no hay mapa guardado valido`.
2. Calibrar sensores. Colocar DOBBY en el punto de referencia físico de salida.
3. Pulsar **APRENDER** y recorrer exactamente una vuelta bajo supervisión.
4. Pulsar **FIN VUELTA**. Debe anunciar `MAPA FLASH: GUARDADO OK + CRC; persiste al apagar`.
5. Pulsar **ESTADO**. Debe mostrar `MAPA estado=2 ... flash=1`.
6. **Detener** y apagar totalmente DOBBY. Encender de nuevo. Después de la calibración inicial, pulsar **ESTADO** y comprobar `MAPA estado=2 ... flash=1`, misma longitud, puntos y porcentaje de lectura. Si no ocurre, NO competir con el replay y recuperar el firmware previo.
7. Sin pulsar APRENDER de nuevo, colocar DOBBY en **el mismo punto y orientación** que el aprendizaje. Probar **MAPA SEGURO** con STOP preparado.
8. Mantener **MAPA RÁPIDO** deshabilitado en las pruebas si ha causado salidas de pista. El guardado no modifica la dinámica respecto a V5.2.

## Mensajes

- `MAPA FLASH: GUARDADO OK + CRC`: la escritura pasó la verificación.
- `MAPA FLASH: ERROR DE GUARDADO`: Flash no aceptó la escritura o falló la verificación. El mapa sigue solo en RAM y se perderá al apagar.
- `MAPA FLASH: cargado automaticamente al arrancar`: el micro recuperó el mapa válido tras reiniciarse.
- `MAPA estado=2 ... flash=1`: mapa válido en RAM y copia válida en Flash.
- `MAPA estado=0 ... flash=0`: no hay mapa disponible/validado en esta arranque.

## Comandos de diagnóstico (UART)

- `MAPSTATUS/`: estado y verificación Flash.
- `MAPSAVE/`: guardar explícitamente el mapa válido, solo con robot detenido.
- `MAPLOAD/`: recuperar el mapa Flash, solo con robot detenido.

## Advertencias

- NO usar **Full chip erase**, porque borraría el sector de mapa y el de parámetros.
- Guardar una pista nueva sustituye la anterior. Los mapas viejos no se conservan.
- Una alimentación interrumpida **durante** el guardado puede dejar un mapa inválido: los CRC impiden reproducirlo, pero habría que aprender otra vez.
- No es posible determinar de forma infalible el inicio/final a partir de una línea negra continua: finalizar manualmente al completar el circuito.
- Guardado automático solo al pulsar **FIN VUELTA** con aprendizaje activo y válido; **STOP** aborta sin guardar.
- Es necesario validar por compilación ARM y prueba real de power-cycle antes de usar esta versión en competencia.

## Recuperación V5.2

```powershell
git fetch origin
git switch v5-02-replay-seguro
cube-cmake --build .\build\Debug --clean-first
$LASTEXITCODE
```

Si `v5-02-replay-seguro` no existe localmente: `git switch -c v5-02-replay-seguro --track origin/v5-02-replay-seguro`.
Programar el ELF correspondiente sin borrado completo. **La V5.2 no carga mapas de Flash**, y sus mapas RAM se pierden al reiniciar.
