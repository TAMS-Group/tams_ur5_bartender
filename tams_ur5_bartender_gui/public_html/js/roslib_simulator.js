/* global ros */

(function (global) {
    var cocktailsList = [
        "test libre",
        "medina",
        "pinacolada",
        "pirate's dream",
        "angel",
        "devil's drink",
        "montagna"
    ];
    var ingredients = [
        "Tequila",
        "Tequila",
        "Tequila",
        "Tequila",
        "Tequila#Gin",
        "Tequila",
        "Cola"
    ];
    var bottles = ["Gin", "Tequila", "Cola"];
    var availableList = [];
    var recBottleList = [];
    var recIngredientsList = [];
    // Public Namespace
    global.ROSLIB = {
        Ros: Ros,
        ActionClient: ActionClient,
        Goal: Goal,
        Topic: Topic
    };
    function Ros() {
        startSimulation();
    }
    Ros.prototype = $({});

    function ActionClient() {}
    ActionClient.prototype = {};

    function Goal() {}
    Goal.prototype.on = function (name, callback) {
        var reactions = {
            weights: [0.3, 0.7], // probabilities
            results: [0, 1], // values to return
            success: "",
            endTime: 0,
            "feedback": function () {
                this.success = getRandom(this.weights, this.results);
                var lastFeedback = this.success ? "Finished Mixing!" : "Failed Mixing";
                var feedbacks = [
                    "Planning Path",
                    "1st Bottle Poured",
                    lastFeedback
                ];
                var time = 1000;
                feedbacks.forEach(function (e) {
                    var msg = {task_state: ""};
                    msg.task_state = e;
                    publish(time, msg, callback);
                    time += (Math.floor(Math.random() * 10000) + 3000);
                });
                this.endTime = time;
            },
            "result": function () {
                var msg = {success: ""};
                msg.success = this.success;
                publish(10000, msg, callback);
            }
        };
        reactions[name]();
    };

    function publish(time, msg, callback) {
        setTimeout(function () {
            callback(msg);
        }, time);
    }

    Goal.prototype.send = function () {};
    function Topic() {
        var receiving;
        this.subscribe = function (callback) {

            changeArraysOverTime();
            var time = 1000;
            receiving = setInterval(function () {
                var msg = {
                    cocktails: [
                        "test libre",
                        "medina",
                        "pinacolada",
                        "pirate's dream",
                        "angel",
                        "devil's drink",
                        "montagna"
                    ],
                    available: availableList,
                    recognizedBottles: recBottleList,
                    neededBottles: recIngredientsList
                };
                callback(msg);
            }, time);
        };
        this.unsubscribe = function () {
            clearInterval(receiving);
        };
    }
    Topic.prototype = {};
    //Connect or disconnect after some time
    function startSimulation() {
        var weights = [0.1, 0.9]; // probabilities
        var results = [0, 1]; // values to return
        var connect = getRandom(weights, results);
        var time = Math.floor(Math.random() * 4000) + 1000;
        setTimeout(function () {
            ros.trigger((connect) ? "connection" : "error");
        }, time);
    }

//Change all arrays after some time
    function changeArraysOverTime() {
        var time = 1000;
        var changeTimeOfInterval = function () {
            clearInterval(changeArrays);
            availableList = [];
            recIngredientsList = [];
            generateNewRecognizedBottles();
            //recBottleList = bottles;//for testing hardcode recognized bottles
            for (var i in cocktailsList) {
                var neededBottlesArr = ingredients[i].split("#");
                var connect = true;
                var bottleNameBoolArr = [];
                for (var z in neededBottlesArr) {
                    var bottleName = neededBottlesArr[z];
                    var found = recBottleList.indexOf(bottleName) !== -1;
                    bottleNameBoolArr.push(bottleName + ((found) ? "=1" : "=0"));
                    if (!found) {
                        connect = false;
                    }
                }
                recIngredientsList[i] = bottleNameBoolArr.join("#") + "#";
                availableList.push(connect);
            }
            time = Math.floor(Math.random() * 10000) + 6000;
            changeArrays = setInterval(changeTimeOfInterval, time);
        };
        var changeArrays = setInterval(changeTimeOfInterval, time);
    }

    function generateNewRecognizedBottles() {
        recBottleList = [];
        //Make array, to splice later in order not to pick the same bottle randomly
        var availNumbers = [];
        //Chances that no bottles will be recognized 10%!
        var weights = [0.1];
        for (var z = 0; z < bottles.length; z++) {
            availNumbers.push(z);
            if (z > 0) {
                weights.push(0.9 / (bottles.length - 1));
            }
        }
        //pick random amount of random bottles
        var noOfRecBottles = getRandom(weights, availNumbers);
        for (var i = 0; i < noOfRecBottles; i++) {
            var j = Math.floor(Math.random() * (availNumbers.length - 1)) + 0;
            recBottleList.push(bottles[availNumbers[j]]);
            availNumbers.splice(j, 1);
        }
    }

    function getRandom(weights, results) {
        var num = Math.random();
        var s = 0;
        var lastIndex = weights.length - 1;
        for (var i = 0; i < lastIndex; ++i) {
            s += weights[i];
            if (num < s) {
                return results[i];
            }
        }
        return results[lastIndex];
    }
})(this);