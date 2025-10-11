import {numeric, temperature, humidity, pressure, illuminance} from "zigbee-herdsman-converters/lib/modernExtend";

/** @type{import('zigbee-herdsman-converters/lib/types').DefinitionWithExtend | import('zigbee-herdsman-converters/lib/types').DefinitionWithExtend[]} */

export function windDirection(args = {}) {
    return numeric({
        name: "wind_direction",
        cluster: "genAnalogInput",
        attribute: "presentValue",
        reporting: {min: "10_SECONDS", max: "1_HOUR", change: 1},
        description: "Wind direction in degrees",
        unit: "°",
        valueMin: 0,
        valueMax: 360,
        access: "STATE_GET",
        ...args,
    });
}

export function windSpeed(args = {}) {
    return numeric({
        name: "wind_speed",
        cluster: "msWindSpeed",
        attribute: "measuredValue",
        reporting: {min: "10_SECONDS", max: "1_HOUR", change: 1},
        description: "Measured wind speed value",
        unit: "m/s",
        scale: 100,
        access: "STATE_GET",
        ...args,
    });
}

export default {
    zigbeeModel: ["Weather Station"],
    model: "Weather-Station",
    vendor: "Ozone",
    description: "Weather station",
    extend: [temperature(), humidity(), pressure(), illuminance(), windDirection(), windSpeed()]
};
