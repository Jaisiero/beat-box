# Fragmentos invisibles durante la publicación

## Causa comprobada

Al crecer F9 de 32 a 34 cuerpos tras una rotura, el actualizador de instancias
seguía usando el dispatch indirecto del último paso de simulación: un grupo de
32 hilos. Las instancias 32 y 33 quedaban sin actualizar hasta el siguiente paso.

Esas instancias estaban inicializadas por C++. `get_transform_matrix()` reunía
las columnas de `Quaternion::to_matrix()` cuando los valores guardados en los
vectores x/y/z ya eran las filas requeridas por la representación de Vulkan.
La rotación resultante quedaba transpuesta. La traslación era correcta.
El shader de actualización producía la rotación correcta.

El TLAS podía estar completamente construido y seguir conteniendo la matriz
incorrecta: la búsqueda de candidatos de ray tracing y la intersección manual
con el cuerpo usaban transformaciones distintas. El siguiente paso ampliaba el
dispatch y reparaba las instancias pendientes, explicando su reaparición.

La desaparición también se reprodujo con el `device.wait_idle()` anterior
restaurado. No se corrige añadiendo esa espera. Ambos defectos aparecen ya en el
commit base de PR26, `272284a`; no nacen en la retirada de esa espera.

## Corrección

- C++ conserva las filas de la matriz al escribir los datos de instancia.
- La publicación despacha directamente a partir de `current_rigid_body_count`,
  actualizado tras roturas, bajas y spawns. Ya no depende del número de grupos
  calculado antes de modificar la escena.
- No cambian la matemática del solver ni los algoritmos SDF.

## Comprobaciones

El test de CPU aplica los doce floats de la matriz como tres filas de Vulkan y
compara el resultado con una rotación independiente `q * punto * conjugate(q)`.
Incluye identidad, giro de 90 grados, rotación general, traslación y varios
puntos. Detectó diez discrepancias antes del cambio y ninguna después.

La instrumentación temporal comparó los AABBs realmente generados en GPU de los
BLAS conservados; no encontró cambios incorrectos en la cadena examinada.
Después comparó el mapa de cuerpos, direcciones BLAS, máscaras y matrices de las
instancias publicadas. Con el oráculo de matriz corregido, detectó exactamente
las instancias 32 y 33 al pasar de 32 a 34 cuerpos. Sus rotaciones eran la
traspuesta de las esperadas. Evidencia en
`/root/beat-box/work/vanishing/coverage-verify.log`.

La instrumentación añade readbacks y no forma parte del ejecutable final.
Los logs y el patch de diagnóstico se conservan en el directorio de trabajo.
La validación de física por sí sola no comprueba la visibilidad del render.

Tras la corrección, una cadena de F9 volvió a cruzar 32 -> 34 y alcanzó 56 cuerpos
sin ninguna discrepancia de instancias ni geometría en la instrumentación
(`fixed-verify.log`). Los cinco tests de CTest pasan, incluido el test que fallaba
antes del cambio.

El ejecutable final, sin instrumentación, completó la fixture de fractura (900
pasos) y F7 (1800 pasos) con AVBD y TGS, bajo synchronization validation Vulkan:
los cuatro CSV y todos los checkpoints DET coinciden con los controles de PR25,
sin errores reportados. Logs `final-*.log`, `final-*.csv` y `final-checks.log`.
