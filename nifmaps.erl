-module(nifmaps).

-export([
	init/0, createmap/3, destory/1, findpath/5
	, buildmap/3, buildmap/4, can_walk/3
  , free/0
	]).

init() ->
      erlang:load_nif("./libNifmaps", 0).

free() ->
      "NIF library not loaded".

createmap(_Bid, _BlocksLen, _Blocks) ->
    "NIF library not loaded".

% nifmaps:buildmap(2, [[{20.0, 20.0}, {700.0, 20.0}, {700.0, 500.0}, {20.0, 500.0}],[{362.0, 268.0}, {208.0, 334.0}, {385.0, 371.0}, {478.0, 334.0}, {486.0, 259.0}, {400.0, 300.0}], [{217.0, 193.0}, {393.0, 118.0}, {453.0, 315.0}, {311.0, 296.0}]], 1).
%% @doc bid | int 场景bid
%% @doc blocks | List 障碍物信息
%% @doc isCW | 0:逆时针	1:顺时针(影响三角形的生成)
buildmap(_Bid, _Blocks, _IsCW) ->
	"NIF library not loaded".

buildmap(_Bid, _Walls, _Blocks, _IsCW) ->
  "NIF library not loaded".

destory(_Bid) ->
    "NIF library not loaded".

findpath(_Bid, _StartX, _StartY, _EndX, _EndY) ->
    "NIF library not loaded".

%% @doc 检测是否能行走
%% @spec can_walk(x, y) -> 1｜0
can_walk(_Bid, _x, _y) ->
    "NIF library not loaded".