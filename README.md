# testWorkspace
git submodule init \
git submodule update\
conda env create environment.yml \
conda activate testWorkspace \
cd testWorkspace/kdl_parser/kdl_parser_py \
pip3 install . \
cd testWorkspace \ 
python3 test.py \

Eğer aşağıdaki gibi bir hata çıkarsa,

File "test.py", line 5, in <module>
    import PyKDL
ImportError: dynamic module does not define module export function (PyInit_PyKDL)

unset PYTHONPATH  yazıp \
ve test.py'in 2.line'ı PyKDL modülünün pathini yazmanız gerekebilir.
