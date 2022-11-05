virtualenv .env


. $(pwd)/.env/bin/activate

# Dev dependencies
pip install maturin
cd ../dora/apis/python/node
python3 -m maturin develop
cd ../../../../dora-rover

# Dependencies
pip install --upgrade pip
pip install -r requirements.txt

