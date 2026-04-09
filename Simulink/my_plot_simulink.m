function my_plot_simulink(selected_vars, varargin)
% EJEMPLO DE USO
% Primero correr el programa de simulink y exportar las variables a
% graficar con el bloque To Workspace
%
% Ejemplo de un llamado completo a esta función para graficar en una
% sola figura:
% my_plot({'theta_m', 'v_q'}, 'color', {'b', 'r'}, 'layout', 'subplotV', 'lineWidth', [2, 1.5], 'lineStyle', {'-', '--'}, 'title', {'Gráfico de Theta', 'Gráfico de Velocidad'}, 'ylabel', 'theta');
% OPCIONES
% 'layout','subplotV' para graficar subplots verticales
% 'layout','subplotH' para graficar suplots horizontales
% 'layout'.'unico' para graficar todo en un solo grafico


    % Diccionario de unidades
    variables_info = struct(... 
        'F', struct('latex', 'F', 'unidad', 'N'), ...
        'x', struct('latex', 'x', 'unidad', 'm'), ...
        'x1', struct('latex', 'x_1', 'unidad', 'm'), ...
        'x2', struct('latex', 'x_2', 'unidad', 'm'), ...
        'y', struct('latex', 'y', 'unidad', 'm'), ...
        'd_ang', struct('latex', 'd', 'unidad', 'deg'), ...
        'q_consigna', struct('latex', 'q^*', 'unidad', 'rad'), ...
        'q_vel_consigna', struct('latex', 'q^{´*}', 'unidad', 'rad/s'), ...
        'theta', struct('latex', '\theta', 'unidad', 'rad'), ...
        'theta_m', struct('latex', '\theta_m', 'unidad', 'rad'), ...
        'theta_m_consigna', struct('latex', '\theta_m^*', 'unidad', 'rad'), ...
        'theta_observada', struct('latex', '\theta_{obs}', 'unidad', 'rad'), ...
        'delta', struct('latex', '\delta', 'unidad', 'deg'), ...
        'delta_ref', struct('latex', '\delta_{ref}', 'unidad', 'deg'), ...
        'delta_simscape', struct('latex', '\delta \, simscape', 'unidad', 'rad'), ...
        'delta_SS', struct('latex', '\delta \, SS', 'unidad', 'rad'), ...
        'w', struct('latex', '\omega', 'unidad', 'rad/s'), ...
        'w_m', struct('latex', '\omega_m', 'unidad', 'rad/s'), ...
        'w_m_consigna', struct('latex', '\omega_m^*', 'unidad', 'rad/s'), ...
        'w_observada', struct('latex', '\omega_{obs}', 'unidad', 'rad/s'), ...
        'v', struct('latex', 'v', 'unidad', 'm/s'), ...
        'V', struct('latex', 'V', 'unidad', 'V'), ...
        'Vm', struct('latex', 'V_m', 'unidad', 'V'), ...
        'v_q', struct('latex', 'v_q', 'unidad', 'V'), ...
        'v_d', struct('latex', 'v_d', 'unidad', 'V'), ...
        'v_0', struct('latex', 'v_0', 'unidad', 'V'), ...
        'v_a', struct('latex', 'v_a', 'unidad', 'V'), ...
        'v_b', struct('latex', 'v_b', 'unidad', 'V'), ...
        'v_c', struct('latex', 'v_c', 'unidad', 'V'), ...
        'v_a_prima', struct('latex', 'v_a^´', 'unidad', 'V'), ...
        'v_b_prima', struct('latex', 'v_b^´', 'unidad', 'V'), ...
        'v_c_prima', struct('latex', 'v_c^´', 'unidad', 'V'), ...
        'T', struct('latex', 'T', 'unidad', 'N.m'), ...
        'T_ld', struct('latex', 'T_{ld}', 'unidad', 'N.m'), ...
        'T_m', struct('latex', 'T_{m}', 'unidad', 'N.m'), ...
        'i', struct('latex', 'i', 'unidad', 'A'), ...
        'im', struct('latex', 'i_m', 'unidad', 'A'), ...
        'i1', struct('latex', 'i_1', 'unidad', 'A'), ...
        'i2', struct('latex', 'i_2', 'unidad', 'A'), ...
        'i3', struct('latex', 'i_3', 'unidad', 'A'), ...
        'i_q', struct('latex', 'i_q', 'unidad', 'A'), ...
        'i_d', struct('latex', 'i_d', 'unidad', 'A'), ...
        'i_0', struct('latex', 'i_0', 'unidad', 'A'), ...
        'i_a', struct('latex', 'i_a', 'unidad', 'A'), ...
        'i_b', struct('latex', 'i_b', 'unidad', 'A'), ...
        'i_c', struct('latex', 'i_c', 'unidad', 'A'), ...
        'T_s', struct('latex', 'T_s^\circ', 'unidad', '°C'), ...
        'aceleracion', struct('latex', 'aceleracion', 'unidad', 'm/s²') ...
    );

    % Definir los valores predeterminados
    default_colors = {'b', 'r', 'g', 'c', 'm', 'y', 'k'}; % Colores predeterminados
    default_layout = 'unico'; % Layout por defecto
    default_lineWidth = 1; % Grosor de línea por defecto
    default_lineStyle = '-'; % Estilo de línea por defecto
    default_title = {}; % Títulos predeterminados (vacío)
    default_ylabel = ''; % Etiqueta Y predeterminada (vacío)

    % Parsear las opciones opcionales con inputParser
    p = inputParser;
    addRequired(p, 'selected_vars', @(x) iscell(x) && all(cellfun(@ischar, x)));
    addParameter(p, 'color', default_colors, @(x) iscell(x) && all(cellfun(@ischar, x)));
    addParameter(p, 'layout', default_layout, @(x) ismember(x, {'unico', 'subplotV', 'subplotH'}));
    addParameter(p, 'lineWidth', default_lineWidth, @(x) isnumeric(x) && all(x > 0));
    addParameter(p, 'lineStyle', default_lineStyle, @(x) iscell(x) || ischar(x));
    addParameter(p, 'title', default_title, @(x) iscell(x) || ischar(x));
    addParameter(p, 'ylabel', default_ylabel, @(x) ischar(x));

    % Parsear los argumentos
    parse(p, selected_vars, varargin{:});
    
    % Obtener los valores de los parámetros
    colors = p.Results.color;
    layout = p.Results.layout;
    lineWidth = p.Results.lineWidth;
    lineStyle = p.Results.lineStyle;
    titles = p.Results.title; % Títulos personalizados
    ylabel_text = p.Results.ylabel; % Etiqueta Y personalizada

    % Si no se ha especificado el color, usar los colores predeterminados
    if length(colors) < length(selected_vars)
        colors = repmat(colors, 1, ceil(length(selected_vars) / length(colors)));
        colors = colors(1:length(selected_vars));
    end

    % Si no se ha especificado el estilo de línea, usar el estilo predeterminado
    if ischar(lineStyle)  % Si lineStyle es una cadena única
        lineStyle = repmat({lineStyle}, 1, length(selected_vars));  % Repetirlo para todas las variables
    elseif length(lineStyle) < length(selected_vars)
        lineStyle = repmat(lineStyle, 1, ceil(length(selected_vars) / length(lineStyle)));
        lineStyle = lineStyle(1:length(selected_vars));
    end

    % Si no se ha especificado el grosor de la línea, usar el grosor predeterminado
    if length(lineWidth) < length(selected_vars)
        lineWidth = repmat(lineWidth, 1, ceil(length(selected_vars) / length(lineWidth)));
        lineWidth = lineWidth(1:length(selected_vars));
    end

    % Obtener la variable 'out' desde el workspace global (base)
    out = evalin('base', 'out'); % Acceder a 'out' en el workspace global

    % Obtener nombres de las variables dentro de 'out'
    vars = out.who; % Lista de variables en out

    % Inicializar lista de variables válidas
    valid_vars = {};
    
    % Verificar si las variables seleccionadas están dentro de out y son timeseries
    for i = 1:length(selected_vars)
        if ismember(selected_vars{i}, vars) && isa(out.(selected_vars{i}), 'timeseries')
            valid_vars{end+1} = selected_vars{i}; % Guardar solo las que existen y son timeseries
        else
            warning(['La variable "', selected_vars{i}, '" no existe en out o no es timeseries.']);
        end
    end

    % Verificar cuántas variables hay después del filtrado
    num_vars = length(valid_vars);

    % Manejar casos donde no hay variables válidas
    if num_vars == 0
        error('No se encontraron variables válidas en out para graficar.');
    end

    % Crear figura con fondo blanco
    figure('Color', 'w'); 
    hold on; % Permite graficar múltiples señales en la misma figura

    % Inicializar lista de etiquetas en LaTeX para la leyenda
    legend_labels = {};

    % Verificar layout
    if strcmp(layout, 'unico')
        % Graficar todas las variables en la misma gráfica
        for i = 1:num_vars
            var_name = valid_vars{i}; % Nombre de la variable
            color = colors{i}; % Color asignado
            style = lineStyle{i}; % Estilo de línea asignado
            width = lineWidth(i); % Grosor de línea asignado
            time = out.(var_name).Time; % Extraer el tiempo
            data = out.(var_name).Data; % Extraer los datos
            
            % Graficar la variable con su color, estilo y grosor
            plot(time, data, 'LineWidth', width, 'Color', color, 'LineStyle', style);
            xlabel('t [s]');
            
            % Si se especifica un ylabel para layout 'unico', usar el argumento
            if ~isempty(ylabel_text)
                ylabel(['{' variables_info.(ylabel_text).latex '}  [' variables_info.(ylabel_text).unidad ']']);
            else
                ylabel(['{' variables_info.(var_name).latex '}  [' variables_info.(var_name).unidad ']']);
            end
            
            % Si se especifica un título para el layout 'unico', usar el primero
            if ~isempty(titles)
                title(titles(1));
            end
            
            % Agregar el nombre en LaTeX a la lista de etiquetas de la leyenda
            if isfield(variables_info, var_name)
                legend_labels{end+1} = ['$' variables_info.(var_name).latex '$'];
            else
                legend_labels{end+1} = var_name; % Si no está en el diccionario, usar el nombre original
            end

            % Activar la leyenda con nombres en LaTeX
            legend(legend_labels, 'Location', 'best', 'Interpreter', 'latex');
            
            % Ajustar tamaño de los números en los ejes
            set(gca, 'FontSize', 14);
            
            % Activar la cuadrícula y aumentar su grosor
            grid on;
            set(gca, 'GridLineStyle', '-', 'LineWidth', 1.5);

        end

    elseif strcmp(layout, 'subplotV')
        % Graficar cada variable en un subplot vertical (uno encima del otro)
        for i = 1:num_vars
            var_name = valid_vars{i}; % Nombre de la variable
            color = colors{i}; % Color asignado
            style = lineStyle{i}; % Estilo de línea asignado
            width = lineWidth(i); % Grosor de línea asignado
            time = out.(var_name).Time; % Extraer el tiempo
            data = out.(var_name).Data; % Extraer los datos
            
            subplot(num_vars, 1, i); % Crear un subplot por cada variable
            plot(time, data, 'LineWidth', width, 'Color', color, 'LineStyle', style);
            
            xlabel('t [s]');
            ylabel(['{' variables_info.(var_name).latex '}  [' variables_info.(var_name).unidad ']']);
            
            % Si se especifica un título para el layout 'unico', usar el primero
            if ~isempty(titles)
                title(titles{i});
            end
            
            
            % Activar la leyenda con nombres en LaTeX
            if strcmp(layout, 'unico')
                legend(legend_labels, 'Location', 'best', 'Interpreter', 'latex');
            end
            
            % Ajustar tamaño de los números en los ejes
            set(gca, 'FontSize', 14);
            
            % Activar la cuadrícula y aumentar su grosor
            grid on;
            set(gca, 'GridLineStyle', '-', 'LineWidth', 1.5);

        end

    elseif strcmp(layout, 'subplotH')
        % Graficar cada variable en un subplot horizontal (uno al lado del otro)
        for i = 1:num_vars
            var_name = valid_vars{i}; % Nombre de la variable
            color = colors{i}; % Color asignado
            style = lineStyle{i}; % Estilo de línea asignado
            width = lineWidth(i); % Grosor de línea asignado
            time = out.(var_name).Time; % Extraer el tiempo
            data = out.(var_name).Data; % Extraer los datos
            
            subplot(1, num_vars, i); % Crear un subplot por cada variable
            plot(time, data, 'LineWidth', width, 'Color', color, 'LineStyle', style);
            title(['Gráfico de ' var_name]); % Título del gráfico
            xlabel('t [s]');
            ylabel(['{' variables_info.(var_name).latex '}  [' variables_info.(var_name).unidad ']']);
            
            % Activar la leyenda con nombres en LaTeX
            %legend(legend_labels, 'Location', 'best', 'Interpreter', 'latex');
            
            % Ajustar tamaño de los números en los ejes
            set(gca, 'FontSize', 14);
            
            % Activar la cuadrícula y aumentar su grosor
            grid on;
            set(gca, 'GridLineStyle', '-', 'LineWidth', 1.5);

        end
    end
end
