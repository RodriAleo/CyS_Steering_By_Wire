# Tareas para completar el informe de Steering-by-Wire

Fecha: 2026-05-09

## Contexto leido

Archivos revisados:

- `Informe_CyS_SteeringByWire/main.tex`
- `Informe_CyS_SteeringByWire/Guía de redacción.pdf`
- `AI_HANDOFF.md`
- `Simulink/Modelo_SS.m`
- `AI_Analysis/validacion_motor_steering_by_wire.md`

Estado actual del PDF:

- `Informe_CyS_SteeringByWire/main.pdf` tiene 14 paginas.
- El profesor indico un limite maximo de 20 paginas.
- Quedan aproximadamente 6 paginas utiles para cerrar contenido, resultados, conclusiones y referencias.

## Estructura actual del informe

El `main.tex` ya contiene:

1. Titulo, autor y resumen.
2. Introduccion general a direccion convencional y Steering-by-Wire.
3. Desarrollo.
4. Modelado de la planta:
   - ecuacion electrica del motor;
   - ecuacion mecanica del eje motor;
   - ecuacion de cremallera;
   - ecuacion de rueda;
   - definicion de estados;
   - matrices `A`, `Bc`, `Bd`, `C_p`, `C_s`;
   - tabla de parametros.
5. Limitaciones del modelo.
6. Verificacion contra Simscape.
7. Analisis de estabilidad.
8. Analisis de controlabilidad.
9. Analisis de observabilidad.
10. Diseno LQR.
11. Aparicion de error estacionario ante perturbacion.
12. Modelo de torque autoalineante `T_a`.
13. Controlador LQR con accion integral.
14. Lista informal de fuentes para `T_a`.

## Brechas principales detectadas

### Brechas contra la guia

- El resumen actual esta copiado de la guia y no describe el proyecto.
- Falta explicitar objetivos: objetivo principal y objetivos secundarios.
- Falta una seccion formal de resultados.
- Falta una seccion formal de conclusiones.
- Falta una seccion formal de referencias con `\bibliographystyle` y `\bibliography`, o una lista numerada coherente.
- Falta documentar procesamiento de senial: sensor realista, ruido, discretizacion, retencion y observador.
- Falta discusion final de perturbaciones sobre el sistema completo.
- Las figuras deben tener grilla, unidades, leyendas, captions y referencias en texto.

### Brechas contra el modelo actual

El informe esta desactualizado respecto de `Simulink/Modelo_SS.m`:

- La tabla de parametros del informe usa valores viejos del motor y de geometria.
- El modelo actual usa motor equivalente Mosrac U13060:
  - `Lm = 0.42e-3 H`
  - `Rm = 0.135 ohm`
  - `Jm = 1.0e-3 kg.m^2`
  - `Ke = Kt = 0.6`
  - `Vm_sat = 48 V`
- El modelo actual usa `rL = 0.115 m/rad`, no `0.3`.
- El modelo actual reemplazo `rP` directo por husillo de bolas:
  - `G_bs = 1.0`
  - `lead_bs = 32e-3 m/rev`
  - `rP = lead_bs/(2*pi*G_bs) = 0.00509 m/rad`
- El sensor actual mide `y_c`:
  - `C_s = [0 0 0 1 0 0 0]`
  - el informe todavia habla de `theta_m` en varias partes.
- El analisis de observabilidad tiene una inconsistencia textual: menciona `delta` y luego concluye `theta_m`. Debe quedar como observabilidad desde `y_c`.
- El texto actual desarrolla LQR y luego LQRI; debe quedar claro cual es el controlador final usado en resultados. Si el modelo final usado para validacion es LQR, el apartado LQRI debe pasar a ser historico o descartado; si el final es LQRI, los resultados deben corresponder a LQRI.
- El informe no documenta la validacion final del actuador con referencia trapezoidal.

## Estrategia para no superar 20 paginas

Presupuesto recomendado:

| Bloque | Paginas maximas |
| --- | ---: |
| Resumen + introduccion corregida | 1.5 |
| Modelo actualizado y supuestos | 5.0 |
| Controlabilidad/observabilidad/estabilidad | 2.0 |
| Controlador final + observador/sensor | 3.0 |
| Perturbaciones y actuador | 2.5 |
| Resultados | 3.0 |
| Conclusiones + referencias | 2.0 |
| Total objetivo | 19.0 |

Acciones para ahorrar paginas:

- No incluir codigo MATLAB largo en el cuerpo. Reemplazar el bloque `verbatim` de `CalculoTaPorRueda` por ecuaciones/resumen y dejar el codigo como repositorio o apendice si hace falta.
- Reducir la explicacion de fuentes de `T_a` a 1 parrafo y mover URLs a referencias.
- Evitar repetir matrices completas si ya ocupan mucho espacio; conservar solo las necesarias para trazabilidad.
- Combinar figuras relacionadas en subfiguras.
- Usar una sola tabla final de parametros actualizados.
- Usar una tabla breve para resultados del actuador en vez de muchos graficos.

## Tareas priorizadas

### Prioridad 1: corregir coherencia tecnica del informe

- [ ] Reemplazar el `abstract` por un resumen real de menos de 250 palabras:
  - objetivo del proyecto;
  - modelo de planta SbW;
  - controlador y observador;
  - sensor ruidoso/discretizado;
  - validacion de actuador;
  - resultado principal.
- [ ] Agregar objetivos al final de la introduccion:
  - objetivo principal: modelar y controlar un actuador de ruedas para SbW;
  - objetivos secundarios: modelado en espacio de estados, observabilidad/controlabilidad, diseno de control, sensor, perturbaciones, validacion de motor.
- [ ] Actualizar la descripcion del actuador:
  - reemplazar la narrativa de pinion directo por arquitectura `motor -> acople 1:1 -> husillo de bolas -> tuerca -> cremallera`;
  - explicar que `rP` queda como radio cinematico equivalente.
- [ ] Actualizar la tabla de parametros con los valores finales de `Modelo_SS.m`.
- [ ] Corregir `C_s` y toda referencia al sensor:
  - salida sensada final: `y_c`;
  - unidad: metros;
  - no afirmar que el sensor final mide `theta_m`.
- [ ] Corregir el apartado de observabilidad para que sea consistente:
  - observable desde `y_c`;
  - rango 7 con tolerancia explicita por mal condicionamiento;
  - evitar contradiccion con `delta` o `theta_m`.

### Prioridad 2: cerrar sensor, ruido y Kalman

- [ ] Agregar subseccion `Procesamiento de senial y sensor`.
- [ ] Explicar modelo de sensor:
  - primer orden;
  - muestreo y retencion ZOH;
  - ruido de medicion;
  - posible cuantizacion si se mantiene.
- [ ] Justificar parametros de sensor con `AI_Analysis/simulacion_sensor_steering_by_wire.md`.
- [ ] Incluir valores finales propuestos:
  - `Ts_sensor`;
  - `sigma_yc`;
  - `R = sigma_yc^2`;
  - `tau_sensor`.
- [ ] Documentar Kalman:
  - sistema con medicion `y_c`;
  - uso de `lqe(A, Gw, C_s, Q, R)`;
  - explicar el rol de `Q` y `R`;
  - mostrar una figura de estimacion o, si no entra, una tabla/resumen.

### Prioridad 3: decidir y documentar controlador final

- [ ] Confirmar si el controlador final del informe sera LQR o LQRI.
- [ ] Si el final es LQR:
  - reducir LQRI a una nota historica o eliminarlo;
  - explicar que se uso referencia trapezoidal para evitar demanda impulsiva;
  - mostrar que no queda error estacionario relevante en el caso nominal.
- [ ] Si el final es LQRI:
  - mantener la seccion de accion integral;
  - agregar antiwindup o justificar saturacion;
  - usar resultados finales de LQRI, no de LQR.
- [ ] En cualquiera de los dos casos, indicar que el profesor permitio controlador continuo, pero que el sensor se simula discretizado.

### Prioridad 4: incluir actuador, motor y husillo de bolas

- [ ] Agregar subseccion breve de seleccion/validacion del motor:
  - motor seleccionado: Mosrac U13060 48 V;
  - parametros electricos y mecanicos;
  - limites: `I_cont = 30 A`, `I_peak = 70 A`, `T_cont = 18 N.m`, `T_peak = 40 N.m`, `rpm_max = 800`.
- [ ] Agregar tabla antes/despues de transmision:
  - antes: `rP` directo;
  - despues: `rP = lead_bs/(2*pi*G_bs)`;
  - `lead_bs = 32 mm/rev`;
  - `G_bs = 1`.
- [ ] Agregar validacion de rango nominal:
  - caso escalon como severo;
  - caso trapezoidal como nominal;
  - corriente pico/RMS;
  - torque pico;
  - rpm;
  - tension;
  - carrera `y_c`.
- [ ] Usar datos resumidos de `AI_Analysis/validacion_motor_steering_by_wire.md`.

### Prioridad 5: resultados

- [ ] Crear seccion `Resultados`.
- [ ] Incluir maximo 3 figuras principales:
  1. seguimiento `delta_ref` vs `delta` con referencia trapezoidal;
  2. variables de actuador: `Vm`, `Im`, `Tm`, `rpm_m` o curva torque-velocidad;
  3. respuesta ante perturbaciones/sensor ruidoso.
- [ ] Agregar una tabla de metricas:
  - tiempo de establecimiento aproximado;
  - sobrepico;
  - error final;
  - corriente pico;
  - corriente RMS;
  - torque pico;
  - rpm pico.
- [ ] Discutir explicitamente:
  - el escalon exige corriente pico excesiva;
  - la referencia trapezoidal reduce abruptamente el pico de corriente;
  - el caso trapezoidal queda dentro de limites nominales del motor.

### Prioridad 6: perturbaciones

- [ ] Reemplazar el placeholder `INSERTAR SECCION DE PERTURBACIONES`.
- [ ] Explicar perturbaciones usadas:
  - friccion de cremallera `F_f`;
  - friccion de pivote `T_f`;
  - torque autoalineante `T_a`.
- [ ] Simplificar el bloque largo de codigo de `T_a`:
  - dejar ecuaciones/modelo conceptual;
  - retirar el `verbatim` para ahorrar paginas.
- [ ] Mostrar un resultado de perturbacion sobre el sistema completo:
  - con/sin `T_a`;
  - con/sin friccion;
  - o al menos discusion sobre rechazo de perturbaciones y error estacionario.

### Prioridad 7: referencias

- [ ] Crear `Informe_CyS_SteeringByWire/references.bib`.
- [ ] Reemplazar listas de URLs en texto por citas `\cite{}`.
- [ ] Agregar al final:

```latex
\bibliographystyle{IEEEtranN}
\bibliography{references}
```

- [ ] Referencias minimas:
  - guia de redaccion si se cita;
  - fuente SbW/RWA: Schaeffler, Bosch, ZF o Nexteer;
  - Chung et al. 2024 RWA con belt/ball screw;
  - Mosrac U13060;
  - sensor usado para justificar ruido/resolucion;
  - fuentes de torque autoalineante usadas.

## Orden sugerido de trabajo

1. Actualizar parametros/modelo en `main.tex`.
2. Reescribir resumen e introduccion.
3. Resolver decision final LQR vs LQRI.
4. Agregar sensor/Kalman.
5. Agregar actuador/motor/husillo.
6. Agregar resultados en formato compacto.
7. Agregar conclusiones.
8. Agregar referencias.
9. Compilar y verificar conteo de paginas.
10. Recortar figuras/texto si supera 20 paginas.

## Criterios de aceptacion

- [ ] El PDF final tiene 20 paginas o menos.
- [ ] El resumen describe el proyecto real.
- [ ] La introduccion incluye objetivos.
- [ ] El modelo coincide con `Modelo_SS.m`.
- [ ] El informe dice explicitamente que el sensor mide `y_c`.
- [ ] Se documenta sensor ruidoso y discretizado.
- [ ] Se documenta observador Kalman.
- [ ] Se documenta el controlador final sin contradicciones.
- [ ] Se valida el motor Mosrac U13060 con datos de simulacion.
- [ ] Se muestra que la referencia trapezoidal es el caso nominal admisible.
- [ ] Hay resultados y conclusiones formales.
- [ ] Hay referencias formales citadas en el texto.

