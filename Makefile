tec.up:
	@xhost +
	@docker start tec2
tec.down:
	@xhost +
	@docker stop tec2
tec.restart:
	@xhost +
	@docker restart tec2
tec.shell:
	@xhost +	
	@docker exec -it tec2 bash
tec.build:
	@docker build -t tec2 ./
# tec.intelcreate:
# 	@./runROS2Intel.bash