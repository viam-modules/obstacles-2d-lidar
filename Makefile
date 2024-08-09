module.tar.gz:
	tar czf $@ *.sh .env src/*.py src/pointcloud/*.py requirements.txt 

test:
	python3 -m pytest tests/* 

lint:
	python3 -m pylint src --disable=C0114,W0719,R0902,R0913,R0903,R0801
