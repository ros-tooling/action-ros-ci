import { WebhookPayload } from "@actions/github/lib/interfaces";

// Expecting something like:
//  action-ros-ci-repos-override: https://raw.githubusercontent.com/ros2/ros2/master/ros2.repos
const REGEX_REPOS_FILES_OVERRIDE = /action-ros-ci-repos-override:[ ]*([\S]+)/g;
const REGEX_REPOS_FILES_SUPPLEMENTAL = /action-ros-ci-repos-supplemental:[ ]*([\S]+)/g;

/**
 * Extract captures of all matches.
 *
 * The first dimension contains all the matches and the second
 * dimension contains the captures for the given match.
 *
 * @param content the string in which to search
 * @param regex the regex to apply
 * @returns the array of all captures for all matches
 */
function extractCaptures(content: string, regex: RegExp): string[][] {
	const captures: string[][] = [];
	let matches;
	while ((matches = regex.exec(content)) !== null) {
		const capture = matches.slice(1);
		if (capture.length > 0) {
			captures.push(capture);
		}
	}
	return captures;
}

/**
 * Try to extract PR body from context payload.
 */
function extractPrBody(contextPayload: WebhookPayload): string | undefined {
	const prPayload = contextPayload.pull_request;
	if (!prPayload) {
		return undefined;
	}
	return prPayload.body;
}

/**
 * Get list of repos override files.
 *
 * @param contextPayload the github context payload object
 * @returns an array with all declared repos override files
 */
export function getReposFilesOverride(
	contextPayload: WebhookPayload
): string[] {
	const prBody = extractPrBody(contextPayload);
	if (!prBody) {
		return [];
	}

	return extractCaptures(prBody, REGEX_REPOS_FILES_OVERRIDE).map((capture) => {
		return capture[0];
	});
}

/**
 * Get list of override repos files.
 *
 * @param contextPayload the github context payload object
 * @returns an array with all declared override repos files
 */
export function getReposFilesSupplemental(
	contextPayload: WebhookPayload
): string[] {
	const prBody = extractPrBody(contextPayload);
	if (!prBody) {
		return [];
	}

	return extractCaptures(prBody, REGEX_REPOS_FILES_SUPPLEMENTAL).map(
		(capture) => {
			return capture[0];
		}
	);
}
