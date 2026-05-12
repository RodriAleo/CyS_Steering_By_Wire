$pdf_mode = 1;
$pdflatex = 'pdflatex -interaction=nonstopmode -halt-on-error -synctex=1 %O %S';

# Keep the final PDF in the project directory and move auxiliary files
# to a dedicated build folder to avoid cluttering the source tree.
$out_dir = '.';
$aux_dir = 'build';
$emulate_aux = 1;
