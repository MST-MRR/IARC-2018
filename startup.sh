gunicorn -b localhost:5000 --chdir ./Integration main:api --reload &
python ./Collision/ScanseRun.py & 
echo "Ready to fly..."