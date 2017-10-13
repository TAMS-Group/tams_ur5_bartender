//Zuerst rosbridge starten (in nem launchfile)
//rosbridge
//http://wiki.ros.org/rosbridge_suite
//Dann Html Datei öffnen, die roslibjs enthält, welches mit rosbridge kommunizieren kann
//roslibjs
//http://wiki.ros.org/roslibjs
//über roslibjs Daten hin und her senden (am besten ActionServer aufrufen an einem Controller z.B. mix Cocktail mit Namen "Mojito"

/* global ROSLIB */
$.fn.outerHTML = function () {
    return $('<div />').append(this.eq(0).clone()).html();
};
//Make sure tooltips go away when not hovering anymore
//$('[data-toggle="tooltip"]').tooltip({
//    trigger: 'hover'
//});

$('body').tooltip({selector: '[data-toggle=tooltip]'});
var ros;
var cocktailClient;
var availableCocktailslistener;
var disabledClass = "unclickable";
//Initial connection try
connectToRos();
playAudio("#bar-music", 0.9);
// Event listeners
$("#mixButton").click(function (e) {
    submitDrink();
    $(this).prop('disabled', true);
});
$("#reconnect").click(function (e) {
    $(this).prop('disabled', true);
    connectToRos();
});
$("#musicButton").click(function (e) {
    var musicIconElement = $('#musicOnOff');
    var offIcon = "glyphicon-volume-off";
    var onIcon = "glyphicon-volume-up";
    var isOn = musicIconElement.hasClass(onIcon);
    //Turn music on/off
    unMuteAllAudio(isOn);
    //Change Icon
    if (isOn) {
        musicIconElement.removeClass(onIcon);
        musicIconElement.addClass(offIcon);
    } else {
        musicIconElement.removeClass(offIcon);
        musicIconElement.addClass(onIcon);
    }
});
//Recognized Bottles Dropdown
$("#dropdownMenuButton").parent().on('show.bs.dropdown', function () {
    $("#dropdownMenuButton").addClass("active");
});
$("#dropdownMenuButton").parent().on('hide.bs.dropdown', function () {
    $("#dropdownMenuButton").removeClass("active");
});
function connectToRos() {
    loadingAnimation(true);
    ros = new ROSLIB.Ros({
        url: 'ws://localhost:8080'
    });
    ros.on('connection', function () {
        var msg = 'Connected to websocket server.';
        console.log(msg);
        displayMsg(msg, false, 0);
        $("#mixButton").prop('disabled', false);
        $('#reconnect').fadeOut();
        loadingAnimation(false);
        listenToAvailableCocktails();
    });
    ros.on('error', function (error) {
        var errMsg = 'Error connecting to websocket server: ';
        console.log(errMsg, error);
        displayMsg('Error connecting to websocket server! Click on "' + $("#reconnect").text().trim() + '" to try again!');
        loadingAnimation(false);
        $('#mixButton').prop('disabled', true);
        $('#reconnect').prop('disabled', false);
        $('#reconnect').fadeIn();
    });
    ros.on('close', function () {
        var errMsg = 'Connection to websocket server closed.';
        if ($('#reconnect').prop("display") === "none") {
            $('#reconnect').fadeIn();
        }
        console.log(errMsg);
    });
    cocktailClient = new ROSLIB.ActionClient({
        ros: ros,
        serverName: 'cocktail_mixer',
        actionName: 'tams_ur5_bartender_coordination/CocktailAction'
    });
}

// Subscribing to a Topic
function listenToAvailableCocktails() {

    availableCocktailslistener = new ROSLIB.Topic({
        ros: ros,
        name: 'availableCocktails',
        messageType: 'tams_ur5_bartender_msgs/CocktailList'
    });
    availableCocktailslistener.subscribe(function (message) {
        addCocktails(message.cocktails, message.available, message.recognizedBottles, message.neededBottles);
        return;
    });
}

var animationTimeout; //needed in order to turn it off when finished so that loading bar doesn't suddenly appear later
//When Clicking on MIX IT
function submitDrink() {
    var selectedRadioButton = $("input:radio[name='cocktail']:checked");
    var available = $(selectedRadioButton.parent()).hasClass("darkgreen");
    var drinkName = selectedRadioButton.val();
    //Check if a drink was chosen and is available
    if (available && drinkName) {
        loadingAnimation(true);
        playAudio("#robot-music", 0.9);
        //during pouring there's no need to listen to available cocktails anymore
        availableCocktailslistener.unsubscribe();
        var goal = new ROSLIB.Goal({
            actionClient: cocktailClient,
            goalMessage: {
                cocktail: drinkName
            }
        });
        goal.on('feedback', function (feedback) {
            loadingAnimation(false);
            var msg = feedback.task_state;
            var msgStatus;
            if (new RegExp(["Failed", "FAILED"].join("|")).test(msg)) {
                msgStatus = 1;
            } else if (new RegExp(["Picking up bottle succeeded","Moving bottle to glass succeeded"].join("|")).test(msg)){
                msgStatus = 2;
            } else if (new RegExp(["Finished", "succeeded"].join("|")).test(msg)) {
                msgStatus = 0;
            } else {
                msgStatus = 2;
            }
            console.log(msg);
            displayMsg(msg, false, msgStatus);
            //If still process still ongoing show loading bar after a while
            if (msgStatus >= 2) {
                animationTimeout = setTimeout(function () {
                    loadingAnimation(true);
                }, 2500);
                playRandomNoRepeatSound(feedbackChooser);
            } else {
                var success = msgStatus === 0;
                var chooser = (success) ? successChooser : failChooser;
                playRandomNoRepeatSound(chooser);
            }
        });
        goal.on('result', function (result) {
            playAudio("#bar-music", 1);
            $("#mixButton").prop('disabled', false);
            loadingAnimation(false);
            listenToAvailableCocktails();
            var msg = 'Final Result: ' + result.success;
            console.log(msg);
        });
        goal.send();
    } else {
        var msg = "Please choose an available cocktail first!";
        displayMsg(msg);
        $("#mixButton").prop('disabled', false);
    }
}

function emptyIngredientsDisplay() {
    $("#bottlesOfCocktail").html("Ingredients...");
}

//Add cocktail list to GUI
function addCocktails(cocktailsArray, availabilityArray, recognizedBottles, neededBottlesArray) {
    try {
        addRecognizedBottles(recognizedBottles);
        //Check if Array valid, not empty and an available cocktail exists
        if (cocktailsArray && cocktailsArray.length !== 0 && availabilityArray.indexOf(true) > -1) {
            //If msg "No cocktails..." is displayed, remove that msg
            if ($(".btn-group").text().indexOf("No cocktails available") !== -1) {
                $(".btn-group").empty();
                emptyIngredientsDisplay();
            }
            $("#mixButton").prop('disabled', false);

            var availableIngredientColorClass = "darkgreen";
            var tooltipProperty = "dtool";
            //add neededBottles to name of cocktail
            for (var z = 0; z < cocktailsArray.length; z++) {
                cocktailsArray[z] = cocktailsArray[z] + '#' + neededBottlesArray[z];
            }
            for (var i = 0; i < cocktailsArray.length; i++) {
                //split up long name array = 'name#bottle1=availBool#bottle2=...#' and convert bottles to colored Tooltip
                var nameAndBottlesArr = cocktailsArray[i].split('#');
                var cocktailName = nameAndBottlesArr[0];
                var tooltipElement = $("<div></div>").addClass("ingredients");
                //start from 1 because 0 is cocktail name 
                //stop one before end of array because it's alway empty after last "#"
                for (var u = 1; u < nameAndBottlesArr.length - 1; u++)
                {
                    var bottleAndBool = nameAndBottlesArr[u].split('=');
                    var bottleName = bottleAndBool[0].replace(/\w\S*/g, capitalize);
                    var bottleBool = bottleAndBool[1];
                    var colorClass = (bottleBool === '1') ? availableIngredientColorClass : 'red';
                    //Create one Tooltip-Entry of bottle and availability color and append to tooltip
                    var bottleItem = $("<span></span>").addClass(colorClass).text("• " + bottleName).append($("<br/>"));
                    tooltipElement.append(bottleItem);
                }
                //Title of tooltip "Ingredients x/y" - after bottle iteration for the x value
                var listEntry = $("<span></span>")
                        .text("Ingredients: " + " (" + tooltipElement.children("." + availableIngredientColorClass).length + "/" + (nameAndBottlesArr.length - 2) + ")")
                        .append($("<br/>"));
                tooltipElement.prepend(listEntry);
                //Append Radiobutton for each Cocktail, disable unavailable cocktail-radios
                var alreadyListed = $('.btn-group').find('[value="' + cocktailName + '"]');
                var tooltipHtml = tooltipElement.outerHTML();
                //If cocktail not listed yet add it
                if (alreadyListed.length === 0) {
                    var $radio = $('<input>').attr({
                        type: 'radio',
                        id: 'cocktails',
                        name: 'cocktail',
                        value: cocktailName
                    });
                    var cocktailLabel = cocktailName.replace(/\w\S*/g, capitalize);
                    //Create label + append Tooltip
                    var $label = $('<label data-toggle="tooltip" data-html="true" data-placement="bottom" data-container=".btn-group"></label>')
                            .addClass("btn btn-default")
                            .attr(tooltipProperty, tooltipHtml)
                            .text(cocktailLabel);
                    //Available Cocktails -> green font
                    if (availabilityArray[i]) {
                        $label.addClass("darkgreen");
                    }
                    $label.append($radio);
                    $label.hover(function () {
                        $("#bottlesOfCocktail").html($(this).attr(tooltipProperty));
                    }, function () {
                        emptyIngredientsDisplay();
                    });
                    $(".btn-group").append($label);
                }
                //If already there
                else {
                    var currentLabel = $(alreadyListed[0]).parent();
                    var oldTooltip = currentLabel.attr(tooltipProperty); //"data-original-title"
                    var noChangeInTooltip = oldTooltip === tooltipHtml;
                    //Renew Tooltip and availability
                    if (!noChangeInTooltip) {
                        currentLabel.attr(tooltipProperty, tooltipHtml);
                        //Available Cocktails -> green font if not already there
                        if (availabilityArray[i] && !currentLabel.hasClass("darkgreen")) {
                            currentLabel.addClass("darkgreen");
                        }
                        //If not available AND available before, then show it
                        //Not available Cocktails not "disabled" because otherwise no Tooltip is seen
                        if (!availabilityArray[i] && currentLabel.hasClass("darkgreen")) {
                            currentLabel.removeClass("darkgreen");
                        }
                    }
                }
            }
            //$('[data-toggle="tooltip"]').tooltip(); //activate Tooltips

            //Sort alphabetically first and then by availability
            sortByContent('.btn-group', 'label', 'asc');
            sortByClass('.btn-group', 'label', 'desc', 'darkgreen');
        } else {
            var msg = "No cocktails available at the moment, try putting a few more bottles on the table! ;)";
            displayMsg(msg, '.btn-group');
            emptyIngredientsDisplay();
            $("#mixButton").prop('disabled', true);
        }
    } catch (e) {
        console.log(e.message);
    }
}

function addRecognizedBottles(recognizedBottles) {
    $("#bottleList").empty();
    $("#howManyBottlesFound").text(recognizedBottles.length);
    for (var x in recognizedBottles) {
        $("#bottleList").append("<li>" + recognizedBottles[x].replace(/\w\S*/g, capitalize) + "</li>");
    }
}

function capitalize(txt) {
    return txt.charAt(0).toUpperCase() + txt.substr(1).toLowerCase();
}

/**
 * Default RED-Status
 * @param {string} msg
 * @param {string} optionalField
 * @param {int} status 0 = black,1 = red,2 = green
 * @returns void
 */
function displayMsg(msg, optionalField, status) {
    var msgField = optionalField ? $(optionalField) : $('.msg');
    //add msg color
    if (typeof status === 'undefined') {
        status = 1;
    }
    msg = addStatusClassToString(msg, status);
    //If already there don't do anything
    var sameMsg = (msg === msgField.html());
    if (sameMsg) {
        return false;
    }

    //fade In
    msgField.stop(true, false);
    msgField.animate({opacity: 0}, 200, function () {
        $(this).html(msg);
    }).animate({opacity: 100}, 100);
}

function addStatusClassToString(str, status) {
    statusClass = "";
    switch (status) {
        case 0:
            statusClass = "darkgreen";
            break;
        case 1:
            statusClass = "darkred";
            break;
        case 2:
            statusClass = "black";
            break;
    }
    htmlStr = '<span class="' + statusClass + '">' + str + '</span>';
    return htmlStr;
}

function loadingAnimation(start) {
    if (start) {
        displayMsg("");
        $(".msg").promise().done(function () {
            $(this).addClass("loading-icon").show();
        });
    } else {
        $(".msg").removeClass("loading-icon");
        clearTimeout(animationTimeout);
    }
}

//Sounds and their functions
//---------------------------------------------------------------
var feebackVoices = [
    "Calculating_Path.mp3",
    "Initializing_Movement.mp3",
    "Mix_it_up.mp3",
    "Move_it.mp3"
];
var successVoices = [
    "Know_your_limits.mp3",
    "Cheers.mp3",
    "Here_you_go.mp3",
    "Let's_Drink.mp3",
    "On_me.mp3",
    "Tip.mp3"
];
var failedVoices = [
    "Don't_blame_it_on_me.mp3",
    "I_had_too_much.mp3",
    "Just_a_machine.mp3",
    "Tough_without_brain.mp3"
];
function randomNoRepeats(array) {
    var copy = array.slice(0);
    return function () {
        if (copy.length < 1) {
            copy = array.slice(0);
        }
        var index = Math.floor(Math.random() * copy.length);
        var item = copy[index];
        copy.splice(index, 1);
        return item;
    };
}
var feedbackChooser = randomNoRepeats(feebackVoices);
var failChooser = randomNoRepeats(failedVoices);
var successChooser = randomNoRepeats(successVoices);

var voiceSpokeAtSec = new Date().getTime() / 1000;
function playRandomNoRepeatSound(voiceChooser) {
    //give 3 seconds before playing a new sound
    cTime = new Date().getTime() / 1000;
    if (cTime < (voiceSpokeAtSec + 5))
    {
        return;
    } else {
        voiceSpokeAtSec = cTime;
    }
    var voiceFile = voiceChooser();
    var resultVoice = document.createElement('audio');
    resultVoice.setAttribute('src', 'audio/' + voiceFile);
    resultVoice.play();
}

function playAudio(eName, volume) {
    var audio = $(eName);
    var audioElement = audio[0];
    fadeOutAllAudio(audioElement);
    if (audioElement.paused) {
        audioElement.volume = 0;
        audioElement.currentTime = 0;
        fadeInAudio(audio, volume);
    }
}

function fadeOutAllAudio(audio) {
    $('audio').each(function () {
        if (this.id !== audio.id) {
            fadeOutAudio(this);
        }
    });
}

function fadeOutAudio(audio) {
    var newVolume = 0;
    $(audio).animate({
        volume: newVolume
    }, 1000).promise().done(function () {
        audio.pause();
        audio.currentTime = 0;
    });
}

function fadeInAudio(audio, volume) {
    audio[0].play();
    var newVolume = volume || 1;
    audio.animate({volume: newVolume}, 1000);
}

function unMuteAllAudio(mute) {
    $('audio').each(function () {
        this.muted = mute;
    });
}
//---------------------------------------------------------------

//Sorting children of HTML elements
//---------------------------------------------------------------
function sortByClass(sel, elem, order, className) {
    var isAvailableCllBck = function (el) {
        return $(el).hasClass(className);
    };
    sortMeBy(sel, elem, order, isAvailableCllBck);
}

function sortByAttribute(sel, elem, order, attr) {
    var cllBck = function (el) {
        return $(el).getAttribute(attr);
    };
    sortMeBy(sel, elem, order, cllBck);
}

function sortByContent(sel, elem, order) {
    var cllBck = function (el) {
        return $(el).html();
    };
    sortMeBy(sel, elem, order, cllBck);
}

function sortMeBy(sel, elem, order, cllbck) {
    try {
        //If no special callback defined sort by content
        if (!cllbck) {
            sortByContent(sel, elem, order);
            return;
        }
        var $selector = $(sel);
        var $element = $selector.children(elem);
        $element.sort(function (a, b) {
            var an = cllbck(a);
            var bn = cllbck(b);
            if (order === 'asc') {
                if (an > bn)
                    return 1;
                if (an < bn)
                    return -1;
            } else if (order === 'desc') {
                if (an < bn)
                    return 1;
                if (an > bn)
                    return -1;
            }
            return 0;
        });
        $element.detach().appendTo($selector);
    } catch (e) {
        console.log(e.message);
    }
}
//---------------------------------------------------------------
